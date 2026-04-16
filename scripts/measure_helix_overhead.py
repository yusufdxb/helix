#!/usr/bin/env python3
"""Measure HELIX ROS 2 node overhead: CPU, RSS memory, and /helix/faults output.

Launches HELIX nodes inside this process, runs for a configurable duration,
and samples resource usage every second. Outputs JSON results.

When ``--with-adapter`` is set, the three packaged ``helix_adapter`` lifecycle
nodes (``topic_rate_monitor``, ``json_state_parser``, ``pose_drift_monitor``)
are instantiated alongside the ``helix_core`` nodes and added to the same
executor. They are configured + activated and then drained on shutdown. With
no robot present the adapter rate windows will report NaN — that is the
intended "publisher exists, no data" semantics, not a failure.

Output: ``results/helix_overhead_{nodes_only,with_adapter}_live.json``.
The ``_live`` suffix is deliberate so re-running this script does NOT overwrite
the curated Session 1 evidence at ``results/helix_overhead_with_adapter.json``,
which the docs-consistency tests pin against.

Usage:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 scripts/measure_helix_overhead.py [--duration 60] [--with-adapter]
"""

import argparse
import json
import os
import statistics
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import psutil
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from helix_core.anomaly_detector import AnomalyDetector
from helix_core.heartbeat_monitor import HeartbeatMonitor
from helix_core.log_parser import LogParser
from helix_msgs.msg import FaultEvent

SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = SCRIPT_DIR.parent / "results"


class FaultCounter(Node):
    """Counts FaultEvent messages on /helix/faults."""

    def __init__(self):
        super().__init__("helix_fault_counter")
        self.count = 0
        self.faults = []
        self._sub = self.create_subscription(
            FaultEvent, "/helix/faults", self._on_fault, 100
        )

    def _on_fault(self, msg):
        self.count += 1
        self.faults.append({
            "type": msg.fault_type,
            "node": msg.node_name,
            "detail": msg.detail[:100],
            "time": time.time(),
        })


def sample_process_resources(pid: int) -> dict:
    """Sample CPU and memory for a process."""
    try:
        proc = psutil.Process(pid)
        return {
            "rss_mb": proc.memory_info().rss / (1024 * 1024),
            "cpu_percent": proc.cpu_percent(interval=0.1),
            "num_threads": proc.num_threads(),
        }
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return {"rss_mb": 0, "cpu_percent": 0, "num_threads": 0}


def _build_adapter_nodes():
    """Instantiate the three packaged helix_adapter lifecycle nodes.

    Returns the list of constructed nodes. Raises ImportError if helix_adapter
    is not built / sourced — which is the right failure for ``--with-adapter``
    (refuse to mislabel a measurement that did not actually include an adapter).
    """
    from helix_adapter.json_state_parser import JsonStateParser
    from helix_adapter.pose_drift_monitor import PoseDriftMonitor
    from helix_adapter.topic_rate_monitor import TopicRateMonitor

    return [TopicRateMonitor(), JsonStateParser(), PoseDriftMonitor()]


_HELIX_NODE_NAMES = (
    "helix_log_parser",
    "helix_anomaly_detector",
    "helix_heartbeat_monitor",
    "helix_topic_rate_monitor",
    "helix_json_state_parser",
    "helix_pose_drift_monitor",
)


class CollisionError(RuntimeError):
    """A live ros2 launch already owns one of the helix node names."""


def _check_for_collision() -> list:
    """Return the list of helix_* node names already discoverable on the graph.

    Running this script alongside an active ``ros2 launch`` of the same nodes
    causes name collisions, which in turn corrupt the measurement (publishers
    and subscribers attach to the wrong instances, lifecycle state diverges,
    and the resulting JSON is silently misleading). Detect that condition and
    let the caller refuse to proceed.
    """
    probe = rclpy.create_node("helix_overhead_probe")
    try:
        # ROS 2 graph discovery is asynchronous; give the probe a brief moment
        # to populate before snapshotting.
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(probe, timeout_sec=0.05)
        live = {name for name, _ns in probe.get_node_names_and_namespaces()}
    finally:
        probe.destroy_node()
    return sorted(n for n in _HELIX_NODE_NAMES if n in live)


def run_measurement(duration_sec: int, with_adapter: bool, rules_path: str):
    rclpy.init()

    collisions = _check_for_collision()
    if collisions:
        rclpy.try_shutdown()
        raise CollisionError(
            "refusing to run — these helix node names are already present "
            f"on the ROS 2 graph: {collisions}. Stop the running stack "
            "(ros2 launch helix_bringup helix_sensing.launch.py / "
            "helix_adapter.launch.py) before starting the overhead measurement, "
            "or this script will silently mislabel its output."
        )

    log_parser = LogParser()
    # Apply the resolved rules file to LogParser BEFORE configure, so the
    # measurement actually exercises the regex-matching path instead of running
    # with zero rules and silently mislabeling the result.
    log_parser.set_parameters([
        rclpy.parameter.Parameter(
            "rules_file_path", rclpy.Parameter.Type.STRING, rules_path,
        ),
    ])
    anomaly_detector = AnomalyDetector()
    heartbeat_monitor = HeartbeatMonitor()
    fault_counter = FaultCounter()

    core_nodes = [log_parser, anomaly_detector, heartbeat_monitor]
    adapter_nodes = _build_adapter_nodes() if with_adapter else []
    nodes = core_nodes + adapter_nodes + [fault_counter]

    for node in core_nodes + adapter_nodes:
        node.trigger_configure()
        node.trigger_activate()

    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    pid = os.getpid()
    samples = []

    nodes_running = (
        [n.get_name() for n in core_nodes]
        + [n.get_name() for n in adapter_nodes]
    )
    print(f"HELIX nodes running (PID {pid}), sampling for {duration_sec}s...")
    print(f"  Core nodes: {[n.get_name() for n in core_nodes]}")
    print(f"  Adapter nodes: "
          f"{[n.get_name() for n in adapter_nodes] if adapter_nodes else 'disabled'}")

    t_start = time.time()
    while time.time() - t_start < duration_sec:
        sample = sample_process_resources(pid)
        sample["elapsed_sec"] = round(time.time() - t_start, 1)
        samples.append(sample)
        time.sleep(1.0)

    rss_values = [s["rss_mb"] for s in samples]
    cpu_values = [s["cpu_percent"] for s in samples if s["cpu_percent"] > 0]

    results = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "duration_sec": duration_sec,
        "with_adapter": with_adapter,
        "pid": pid,
        "nodes_running": nodes_running,
        "fault_events_received": fault_counter.count,
        "fault_details": fault_counter.faults[:20],
        "resource_samples": len(samples),
        "rss_mb": {
            "mean": round(statistics.mean(rss_values), 1),
            "max": round(max(rss_values), 1),
            "min": round(min(rss_values), 1),
        },
        "cpu_percent": {
            "mean": round(statistics.mean(cpu_values), 1) if cpu_values else 0,
            "max": round(max(cpu_values), 1) if cpu_values else 0,
        },
        "samples": samples,
    }

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    tag = "with_adapter" if with_adapter else "nodes_only"
    out_path = RESULTS_DIR / f"helix_overhead_{tag}_live.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nResults ({duration_sec}s):")
    print(f"  RSS: {results['rss_mb']['mean']:.1f} MB mean, "
          f"{results['rss_mb']['max']:.1f} MB max")
    print(f"  CPU: {results['cpu_percent']['mean']:.1f}% mean, "
          f"{results['cpu_percent']['max']:.1f}% max")
    print(f"  Faults detected: {fault_counter.count}")
    print(f"  Saved: {out_path}")

    executor.shutdown()
    for node in core_nodes + adapter_nodes:
        try:
            node.trigger_deactivate()
            node.trigger_cleanup()
        except Exception:
            pass
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()

    return results


def main():
    parser = argparse.ArgumentParser(description="Measure HELIX node overhead")
    parser.add_argument(
        "--duration", type=int, default=60,
        help="Measurement duration in seconds",
    )
    parser.add_argument(
        "--with-adapter", action="store_true",
        help=("Instantiate the helix_adapter lifecycle nodes alongside the "
              "helix_core nodes (requires helix_adapter to be built/sourced)"),
    )
    args = parser.parse_args()

    rules_path = str(
        SCRIPT_DIR.parent / "src" / "helix_bringup" / "config" / "log_rules.yaml"
    )
    try:
        run_measurement(args.duration, args.with_adapter, rules_path)
    except ImportError as exc:
        print(
            f"ERROR: --with-adapter requested but helix_adapter is not "
            f"importable ({exc}). Build the workspace and source install/setup.bash, "
            f"then retry. Refusing to silently report a non-adapter measurement.",
            file=sys.stderr,
        )
        rclpy.try_shutdown()
        sys.exit(2)
    except CollisionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)


if __name__ == "__main__":
    main()
