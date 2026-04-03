#!/usr/bin/env python3
"""
Measure HELIX ROS 2 node overhead: CPU, RSS memory, and /helix/faults output.

Launches HELIX nodes inside this process, runs for a configurable duration,
and samples resource usage every second. Outputs JSON results.

Usage:
  source /opt/ros/humble/setup.bash
  source /tmp/helix_ws/install/setup.bash
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

from helix_core.log_parser import LogParser
from helix_core.anomaly_detector import AnomalyDetector
from helix_core.heartbeat_monitor import HeartbeatMonitor
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


def run_measurement(duration_sec: int, with_adapter: bool, rules_path: str):
    rclpy.init()

    # Create nodes
    nodes = []

    log_parser = LogParser()
    anomaly_detector = AnomalyDetector()
    heartbeat_monitor = HeartbeatMonitor()
    fault_counter = FaultCounter()

    nodes = [log_parser, anomaly_detector, heartbeat_monitor, fault_counter]

    # Configure and activate lifecycle nodes
    log_parser.trigger_configure()
    log_parser.trigger_activate()
    anomaly_detector.trigger_configure()
    anomaly_detector.trigger_activate()
    heartbeat_monitor.trigger_configure()
    heartbeat_monitor.trigger_activate()

    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)

    # Spin in background
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    pid = os.getpid()
    samples = []

    print(f"HELIX nodes running (PID {pid}), sampling for {duration_sec}s...")
    print(f"  Nodes: log_parser, anomaly_detector, heartbeat_monitor")
    print(f"  Adapter: {'yes' if with_adapter else 'no'}")

    t_start = time.time()
    while time.time() - t_start < duration_sec:
        sample = sample_process_resources(pid)
        sample["elapsed_sec"] = round(time.time() - t_start, 1)
        samples.append(sample)
        time.sleep(1.0)

    # Collect results
    rss_values = [s["rss_mb"] for s in samples]
    cpu_values = [s["cpu_percent"] for s in samples if s["cpu_percent"] > 0]

    results = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "duration_sec": duration_sec,
        "with_adapter": with_adapter,
        "pid": pid,
        "nodes_running": ["log_parser", "anomaly_detector", "heartbeat_monitor"],
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

    # Save
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    tag = "with_adapter" if with_adapter else "nodes_only"
    out_path = RESULTS_DIR / f"helix_overhead_{tag}.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nResults ({duration_sec}s):")
    print(f"  RSS: {results['rss_mb']['mean']:.1f} MB mean, {results['rss_mb']['max']:.1f} MB max")
    print(f"  CPU: {results['cpu_percent']['mean']:.1f}% mean, {results['cpu_percent']['max']:.1f}% max")
    print(f"  Faults detected: {fault_counter.count}")
    print(f"  Saved: {out_path}")

    executor.shutdown()
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()

    return results


def main():
    parser = argparse.ArgumentParser(description="Measure HELIX node overhead")
    parser.add_argument("--duration", type=int, default=60, help="Measurement duration in seconds")
    parser.add_argument("--with-adapter", action="store_true", help="Include passive adapter")
    args = parser.parse_args()

    rules_path = str(SCRIPT_DIR.parent / "src" / "helix_bringup" / "config" / "log_rules.yaml")
    run_measurement(args.duration, args.with_adapter, rules_path)


if __name__ == "__main__":
    main()
