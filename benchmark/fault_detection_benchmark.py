#!/usr/bin/env python3
"""
HELIX Fault Detection Benchmark
================================
Measures time-to-detection (TTD) latency for each fault type injected by
the HELIX fault injector. Runs the full sensing stack and records:

  - time from fault injection timestamp to FaultEvent publish timestamp
  - false positive rate over a baseline (no-fault) observation window
  - detection rate per fault type

Usage:
  # Terminal 1 — start HELIX sensing stack
  source ~/helix_ws/helix_env.sh
  ros2 launch helix_bringup helix_sensing.launch.py

  # Terminal 2 — run this benchmark
  source ~/helix_ws/helix_env.sh
  python3 benchmark/fault_detection_benchmark.py

  # With custom params:
  python3 benchmark/fault_detection_benchmark.py \\
      --trials 20 \\
      --baseline-sec 60 \\
      --output benchmark/results/run_$(date +%Y%m%d_%H%M).csv

Outputs:
  benchmark/results/<run>.csv  — per-trial TTD values
  benchmark/results/<run>.json — summary statistics
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from helix_msgs.msg import FaultEvent


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class TrialResult:
    trial_id: int
    fault_type: str          # CRASH, ANOMALY, LOG_PATTERN
    injected_at: float       # wall time at injection
    detected_at: Optional[float]  # wall time of FaultEvent publish (None = missed)
    ttd_sec: Optional[float] = None   # detected_at - injected_at
    detected: bool = False

    def __post_init__(self):
        if self.detected_at is not None:
            self.ttd_sec = self.detected_at - self.injected_at
            self.detected = True


@dataclass
class BenchmarkSummary:
    n_trials: int
    fault_types: List[str]
    detection_rate: float
    mean_ttd_sec: Optional[float]
    std_ttd_sec: Optional[float]
    min_ttd_sec: Optional[float]
    max_ttd_sec: Optional[float]
    false_positives_in_baseline: int
    baseline_duration_sec: float
    false_positive_rate_per_min: float
    timestamp: str


# ── Benchmark node ───────────────────────────────────────────────────────────

class BenchmarkCollector(Node):
    """Lightweight ROS 2 node that listens for FaultEvents during benchmarking."""

    def __init__(self):
        super().__init__("helix_benchmark_collector")
        self._fault_events: List[FaultEvent] = []
        self._create_subscription(
            FaultEvent, "/helix/faults",
            lambda msg: self._fault_events.append(msg), 10
        )

    def _create_subscription(self, msg_type, topic, callback, qos):
        self.create_subscription(msg_type, topic, callback, qos)

    def drain(self) -> List[FaultEvent]:
        """Return and clear collected events."""
        evts = list(self._fault_events)
        self._fault_events.clear()
        return evts

    def spin_for(self, duration_sec: float):
        """Spin for `duration_sec` seconds, collecting events."""
        deadline = time.time() + duration_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)


class FaultInjectorClient(Node):
    """Publishes fault injection triggers matching helix_bringup/fault_injector protocol."""

    # Topic the fault injector node subscribes to for external triggers.
    # Set HELIX_INJECT_TOPIC env var to override.
    INJECT_TOPIC = os.environ.get("HELIX_INJECT_TOPIC", "/helix/inject_fault")

    def __init__(self):
        super().__init__("helix_benchmark_injector")
        self._pub = self.create_publisher(String, self.INJECT_TOPIC, 10)
        # Give the publisher time to connect
        time.sleep(0.3)

    def inject(self, fault_type: str) -> float:
        """
        Publish a fault injection command and return the injection wall time.

        Args:
            fault_type: One of "CRASH", "ANOMALY", "LOG_PATTERN"

        Returns:
            Wall time at injection (seconds since epoch).
        """
        msg = String()
        msg.data = fault_type
        inject_time = time.time()
        self._pub.publish(msg)
        return inject_time


# ── Benchmark runner ─────────────────────────────────────────────────────────

def run_baseline(collector: BenchmarkCollector, duration_sec: float) -> int:
    """
    Observe for `duration_sec` without injecting faults.
    Returns count of FaultEvents received (false positives).
    """
    print(f"  [baseline] observing for {duration_sec:.0f}s with no injected faults...")
    collector.drain()
    collector.spin_for(duration_sec)
    fp_events = collector.drain()
    print(f"  [baseline] {len(fp_events)} false positive(s) detected")
    return len(fp_events)


def run_trial(
    injector: FaultInjectorClient,
    collector: BenchmarkCollector,
    trial_id: int,
    fault_type: str,
    wait_sec: float = 5.0,
    cooldown_sec: float = 2.0,
) -> TrialResult:
    """
    Inject one fault, wait `wait_sec` for detection, return TrialResult.
    """
    collector.drain()  # clear stale events

    print(f"  [trial {trial_id}] injecting {fault_type}...", end=" ", flush=True)
    injected_at = injector.inject(fault_type)

    collector.spin_for(wait_sec)
    events = collector.drain()

    # Match first FaultEvent with the correct fault_type
    match: Optional[FaultEvent] = None
    for evt in events:
        if evt.fault_type == fault_type:
            match = evt
            break

    detected_at = match.timestamp if match else None
    result = TrialResult(
        trial_id=trial_id,
        fault_type=fault_type,
        injected_at=injected_at,
        detected_at=detected_at,
    )

    status = f"DETECTED in {result.ttd_sec:.3f}s" if result.detected else "MISSED"
    print(status)

    # Cooldown between injections
    time.sleep(cooldown_sec)
    return result


def compute_summary(
    results: List[TrialResult],
    false_positives: int,
    baseline_sec: float,
) -> BenchmarkSummary:
    detected = [r for r in results if r.detected]
    ttds = [r.ttd_sec for r in detected]

    mean_ttd = sum(ttds) / len(ttds) if ttds else None
    std_ttd = None
    if len(ttds) > 1:
        var = sum((x - mean_ttd) ** 2 for x in ttds) / len(ttds)
        std_ttd = math.sqrt(var)

    fp_rate = (false_positives / baseline_sec) * 60 if baseline_sec > 0 else 0.0

    return BenchmarkSummary(
        n_trials=len(results),
        fault_types=sorted(set(r.fault_type for r in results)),
        detection_rate=len(detected) / len(results) if results else 0.0,
        mean_ttd_sec=mean_ttd,
        std_ttd_sec=std_ttd,
        min_ttd_sec=min(ttds) if ttds else None,
        max_ttd_sec=max(ttds) if ttds else None,
        false_positives_in_baseline=false_positives,
        baseline_duration_sec=baseline_sec,
        false_positive_rate_per_min=fp_rate,
        timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
    )


def save_results(
    results: List[TrialResult],
    summary: BenchmarkSummary,
    output_path: str,
):
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    # CSV per-trial
    csv_path = out.with_suffix(".csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["trial_id", "fault_type",
                                               "injected_at", "detected_at",
                                               "ttd_sec", "detected"])
        writer.writeheader()
        for r in results:
            writer.writerow(asdict(r))
    print(f"\n  Per-trial CSV: {csv_path}")

    # JSON summary
    json_path = out.with_suffix(".json")
    with open(json_path, "w") as f:
        json.dump(asdict(summary), f, indent=2, default=str)
    print(f"  Summary JSON: {json_path}")


def print_summary(s: BenchmarkSummary):
    print("\n" + "=" * 60)
    print("HELIX FAULT DETECTION BENCHMARK — RESULTS")
    print("=" * 60)
    print(f"  Trials         : {s.n_trials}")
    print(f"  Fault types    : {', '.join(s.fault_types)}")
    print(f"  Detection rate : {s.detection_rate * 100:.1f}%")
    if s.mean_ttd_sec is not None:
        print(f"  Mean TTD       : {s.mean_ttd_sec * 1000:.0f} ms")
        if s.std_ttd_sec is not None:
            print(f"  Std TTD        : {s.std_ttd_sec * 1000:.0f} ms")
        print(f"  Min / Max TTD  : {s.min_ttd_sec * 1000:.0f} ms / {s.max_ttd_sec * 1000:.0f} ms")
    else:
        print("  TTD            : N/A (no detections)")
    print(f"  False positives: {s.false_positives_in_baseline} in {s.baseline_duration_sec:.0f}s "
          f"({s.false_positive_rate_per_min:.2f}/min)")
    print("=" * 60)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="HELIX fault detection benchmark")
    parser.add_argument("--trials", type=int, default=10,
                        help="Number of fault injection trials (default: 10)")
    parser.add_argument("--baseline-sec", type=float, default=30.0,
                        help="Baseline observation window in seconds (default: 30)")
    parser.add_argument("--wait-sec", type=float, default=5.0,
                        help="Seconds to wait for detection per trial (default: 5)")
    parser.add_argument("--fault-types", nargs="+",
                        default=["CRASH", "ANOMALY", "LOG_PATTERN"],
                        help="Fault types to cycle through")
    parser.add_argument("--output", type=str,
                        default=f"benchmark/results/run_{time.strftime('%Y%m%d_%H%M')}",
                        help="Output file base path (no extension)")
    args = parser.parse_args()

    rclpy.init()
    collector = BenchmarkCollector()
    injector = FaultInjectorClient()

    print("HELIX Fault Detection Benchmark")
    print(f"  Trials: {args.trials}  |  Fault types: {args.fault_types}")
    print(f"  Baseline: {args.baseline_sec}s  |  Detection window: {args.wait_sec}s/trial\n")

    # Baseline pass
    false_positives = run_baseline(collector, args.baseline_sec)

    # Injection trials — cycle through fault types
    results: List[TrialResult] = []
    for i in range(args.trials):
        ft = args.fault_types[i % len(args.fault_types)]
        result = run_trial(injector, collector, i + 1, ft, wait_sec=args.wait_sec)
        results.append(result)

    summary = compute_summary(results, false_positives, args.baseline_sec)
    print_summary(summary)
    save_results(results, summary, args.output)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
