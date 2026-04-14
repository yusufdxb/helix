#!/usr/bin/env python3
"""
HELIX end-to-end latency benchmark.

Measures end-to-end latency through the actual ROS 2 callback path:
  publish Float64MultiArray on /helix/metrics
  → AnomalyDetector._on_metric() callback
  → _process_sample() Z-score computation
  → FaultEvent published on /helix/faults
  → subscriber callback records arrival time

Includes: intra-process message passing, callback scheduling, Z-score computation
Excludes: inter-process DDS serialization (single-process executor), network transport

Requires: source /opt/ros/humble/setup.bash && source helix_ws/install/setup.bash
"""

import json
import math
import os
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from helix_msgs.msg import FaultEvent

from helix_core.anomaly_detector import AnomalyDetector

# ── Configuration ───────────────────────────────────────────────────────────
N_TRIALS = 50
WINDOW_SIZE = 60
ZSCORE_THRESHOLD = 3.0
CONSECUTIVE_TRIGGER = 3
SPIN_TIMEOUT = 0.02
MAX_SPIN_AFTER_SPIKES = 100
BASELINE_VALUE = 10.0
SPIKE_VALUE = 100.0

SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = SCRIPT_DIR.parent / "results"


class MetricPublisher(Node):
    """Publishes Float64MultiArray messages on /helix/metrics."""

    def __init__(self):
        super().__init__("bench_metric_publisher")
        self._pub = self.create_publisher(Float64MultiArray, "/helix/metrics", 100)

    def publish_sample(self, metric_name: str, value: float):
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = metric_name
        dim.size = 1
        dim.stride = 1
        msg.layout.dim = [dim]
        msg.data = [value]
        self._pub.publish(msg)


class FaultSubscriber(Node):
    """Subscribes to /helix/faults and records arrival timestamps."""

    def __init__(self):
        super().__init__("bench_fault_subscriber")
        self.last_fault_time = None
        self.last_fault_msg = None
        self._sub = self.create_subscription(
            FaultEvent, "/helix/faults", self._on_fault, 100
        )

    def _on_fault(self, msg: FaultEvent):
        self.last_fault_time = time.perf_counter()
        self.last_fault_msg = msg

    def reset(self):
        self.last_fault_time = None
        self.last_fault_msg = None


def build_cyclic_baseline(n: int, base: float = BASELINE_VALUE) -> list:
    """Generate n baseline samples with small cyclic noise."""
    return [base + 0.5 * math.sin(2 * math.pi * i / n) for i in range(n)]


def run_benchmark():
    rclpy.init()

    # Create nodes
    detector = AnomalyDetector()
    publisher = MetricPublisher()
    subscriber = FaultSubscriber()

    # Configure and activate the lifecycle node
    detector.trigger_configure()
    detector.trigger_activate()

    # Single-threaded executor with all nodes
    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(publisher)
    executor.add_node(subscriber)

    latencies = []
    n_detected = 0

    print(f"Running {N_TRIALS} end-to-end latency trials...")
    print(f"  window_size={WINDOW_SIZE}, zscore_threshold={ZSCORE_THRESHOLD}, "
          f"consecutive_trigger={CONSECUTIVE_TRIGGER}")
    print()

    try:
        for trial in range(N_TRIALS):
            metric_name = f"metric_trial_{trial}"
            subscriber.reset()

            # (a) Feed baseline samples to build the window
            baseline = build_cyclic_baseline(WINDOW_SIZE)
            for sample in baseline:
                publisher.publish_sample(metric_name, sample)
                executor.spin_once(timeout_sec=SPIN_TIMEOUT)

            # (b) Record start time
            t_start = time.perf_counter()

            # (c) Publish 3 consecutive spike samples
            for spike_idx in range(CONSECUTIVE_TRIGGER):
                publisher.publish_sample(metric_name, SPIKE_VALUE)
                executor.spin_once(timeout_sec=SPIN_TIMEOUT)

                # Check early if fault arrived
                if subscriber.last_fault_time is not None:
                    break

            # (d) Spin until FaultEvent arrives or give up
            if subscriber.last_fault_time is None:
                for _ in range(MAX_SPIN_AFTER_SPIKES):
                    executor.spin_once(timeout_sec=SPIN_TIMEOUT)
                    if subscriber.last_fault_time is not None:
                        break

            # (e) Record result
            if subscriber.last_fault_time is not None:
                latency = (subscriber.last_fault_time - t_start) * 1000  # ms
                latencies.append(latency)
                n_detected += 1
                status = f"{latency:8.3f} ms"
            else:
                status = "  NO FAULT"

            print(f"  Trial {trial:3d}: {status}")

    finally:
        executor.shutdown()
        detector.destroy_node()
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

    # ── Compute statistics ──────────────────────────────────────────────────
    if not latencies:
        print("\nNo faults detected across any trial. Cannot compute statistics.")
        return

    latencies_sorted = sorted(latencies)
    mean_ms = statistics.mean(latencies)
    p50 = latencies_sorted[len(latencies_sorted) * 50 // 100]
    p95 = latencies_sorted[min(len(latencies_sorted) - 1, len(latencies_sorted) * 95 // 100)]
    p99 = latencies_sorted[min(len(latencies_sorted) - 1, len(latencies_sorted) * 99 // 100)]
    min_ms = min(latencies)
    max_ms = max(latencies)

    print(f"\n{'='*60}")
    print(f"  End-to-end latency results ({n_detected}/{N_TRIALS} detected)")
    print(f"{'='*60}")
    print(f"  Mean:  {mean_ms:8.3f} ms")
    print(f"  P50:   {p50:8.3f} ms")
    print(f"  P95:   {p95:8.3f} ms")
    print(f"  P99:   {p99:8.3f} ms")
    print(f"  Min:   {min_ms:8.3f} ms")
    print(f"  Max:   {max_ms:8.3f} ms")
    print(f"{'='*60}")

    # ── Save JSON results ───────────────────────────────────────────────────
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    output_path = RESULTS_DIR / "e2e_latency_results.json"

    results = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "description": (
            "End-to-end latency from publishing Float64MultiArray on /helix/metrics "
            "through AnomalyDetector Z-score computation to receiving FaultEvent on "
            "/helix/faults. Includes intra-process message passing, callback scheduling, "
            "and Z-score computation. Excludes inter-process DDS serialization and "
            "network transport (single-process executor)."
        ),
        "n_trials": N_TRIALS,
        "n_detected": n_detected,
        "config": {
            "zscore_threshold": ZSCORE_THRESHOLD,
            "consecutive_trigger": CONSECUTIVE_TRIGGER,
            "window_size": WINDOW_SIZE,
        },
        "latency_ms": {
            "mean": round(mean_ms, 4),
            "p50": round(p50, 4),
            "p95": round(p95, 4),
            "p99": round(p99, 4),
            "min": round(min_ms, 4),
            "max": round(max_ms, 4),
        },
        "all_latencies_ms": [round(l, 4) for l in latencies],
    }

    with open(output_path, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nResults saved to {output_path}")


if __name__ == "__main__":
    run_benchmark()
