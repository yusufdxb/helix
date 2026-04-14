#!/usr/bin/env python3
"""
Analyze per-topic rate statistics from ROS 2 bag files.

Computes rolling rate, jitter, gaps, and stability metrics for each topic
in a bag. Supports comparing stability across multiple bags.

Usage:
  source /opt/ros/humble/setup.bash
  python3 scripts/bag_rate_analysis.py <bag_path> [<bag_path2> ...]
  python3 scripts/bag_rate_analysis.py --compare bag1/ bag2/ bag3/
"""

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag_timestamps(bag_path: str) -> dict:
    """Read all message timestamps per topic from a bag."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {}
    for info in reader.get_all_topics_and_types():
        topic_types[info.name] = info.type

    # Collect timestamps per topic
    timestamps = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in timestamps:
            timestamps[topic] = []
        timestamps[topic].append(t_ns / 1e9)  # convert to seconds

    return timestamps, topic_types


def compute_rate_stats(times: list) -> dict:
    """Compute rate statistics from a list of timestamps."""
    if len(times) < 2:
        return {
            "count": len(times),
            "duration_sec": 0,
            "mean_hz": 0,
            "std_hz": 0,
            "min_gap_ms": 0,
            "max_gap_ms": 0,
            "mean_gap_ms": 0,
            "jitter_ms": 0,
            "stability_pct": 0,
        }

    times_sorted = sorted(times)
    gaps = [times_sorted[i + 1] - times_sorted[i] for i in range(len(times_sorted) - 1)]
    duration = times_sorted[-1] - times_sorted[0]

    mean_gap = sum(gaps) / len(gaps)
    mean_hz = 1.0 / mean_gap if mean_gap > 0 else 0

    # Jitter = std dev of inter-message gaps
    variance = sum((g - mean_gap) ** 2 for g in gaps) / len(gaps)
    jitter = math.sqrt(variance)

    # Stability = percentage of gaps within 20% of mean
    if mean_gap > 0:
        within_bounds = sum(1 for g in gaps if abs(g - mean_gap) / mean_gap < 0.2)
        stability = 100.0 * within_bounds / len(gaps)
    else:
        stability = 0

    # Rolling Hz in 1-second windows
    rolling_hz = []
    window_sec = 1.0
    t0 = times_sorted[0]
    window_start = t0
    window_count = 0
    for t in times_sorted:
        if t - window_start < window_sec:
            window_count += 1
        else:
            if window_count > 0:
                rolling_hz.append(window_count / (t - window_start))
            window_start = t
            window_count = 1

    std_hz = 0
    if len(rolling_hz) >= 2:
        rmean = sum(rolling_hz) / len(rolling_hz)
        std_hz = math.sqrt(sum((r - rmean) ** 2 for r in rolling_hz) / len(rolling_hz))

    return {
        "count": len(times),
        "duration_sec": round(duration, 2),
        "mean_hz": round(mean_hz, 2),
        "std_hz": round(std_hz, 3),
        "min_gap_ms": round(min(gaps) * 1000, 3),
        "max_gap_ms": round(max(gaps) * 1000, 3),
        "mean_gap_ms": round(mean_gap * 1000, 3),
        "jitter_ms": round(jitter * 1000, 3),
        "stability_pct": round(stability, 1),
    }


def analyze_bag(bag_path: str) -> dict:
    """Full rate analysis of a single bag."""
    timestamps, topic_types = read_bag_timestamps(bag_path)

    results = {}
    for topic, times in sorted(timestamps.items()):
        stats = compute_rate_stats(times)
        stats["type"] = topic_types.get(topic, "unknown")
        results[topic] = stats

    return results


def compare_bags(bag_paths: list) -> dict:
    """Compare rate stability across multiple bags."""
    all_results = {}
    for path in bag_paths:
        name = Path(path).name
        all_results[name] = analyze_bag(path)

    # Find common topics
    all_topics = set()
    for result in all_results.values():
        all_topics.update(result.keys())

    comparison = {}
    for topic in sorted(all_topics):
        rates = []
        for name, result in all_results.items():
            if topic in result:
                rates.append({
                    "bag": name,
                    "mean_hz": result[topic]["mean_hz"],
                    "stability_pct": result[topic]["stability_pct"],
                    "jitter_ms": result[topic]["jitter_ms"],
                    "count": result[topic]["count"],
                })

        if len(rates) >= 2:
            hz_values = [r["mean_hz"] for r in rates]
            mean_hz = sum(hz_values) / len(hz_values)
            if mean_hz > 0:
                cv = math.sqrt(sum((h - mean_hz) ** 2 for h in hz_values) / len(hz_values)) / mean_hz
            else:
                cv = 0
            comparison[topic] = {
                "bags": rates,
                "cross_bag_mean_hz": round(mean_hz, 2),
                "cross_bag_cv": round(cv, 4),
                "cross_bag_stable": cv < 0.05,
            }

    return comparison


def main():
    parser = argparse.ArgumentParser(description="Bag rate analysis")
    parser.add_argument("bags", nargs="+", help="Bag paths")
    parser.add_argument("--compare", action="store_true", help="Compare across bags")
    parser.add_argument("--output", "-o", help="Output JSON path")
    args = parser.parse_args()

    if args.compare and len(args.bags) >= 2:
        results = compare_bags(args.bags)
        mode = "comparison"
    else:
        results = analyze_bag(args.bags[0])
        mode = "single"

    output = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "mode": mode,
        "bags": args.bags,
        "results": results,
    }

    # Print summary
    if mode == "single":
        print(f"Bag: {args.bags[0]}")
        print(f"{'Topic':<40} {'Count':>6} {'Hz':>8} {'Jitter':>10} {'Stable':>8}")
        print("-" * 78)
        for topic, stats in sorted(results.items(), key=lambda x: -x[1]["count"]):
            print(f"{topic:<40} {stats['count']:>6} {stats['mean_hz']:>8.1f} "
                  f"{stats['jitter_ms']:>8.2f}ms {stats['stability_pct']:>6.1f}%")
    else:
        print(f"Cross-bag comparison ({len(args.bags)} bags)")
        print(f"{'Topic':<40} {'Mean Hz':>8} {'CV':>8} {'Stable':>8}")
        print("-" * 68)
        for topic, data in sorted(results.items()):
            stable = "YES" if data["cross_bag_stable"] else "NO"
            print(f"{topic:<40} {data['cross_bag_mean_hz']:>8.1f} "
                  f"{data['cross_bag_cv']:>8.4f} {stable:>8}")

    # Save
    if args.output:
        out_path = Path(args.output)
    else:
        out_path = Path(__file__).resolve().parent.parent / "results" / f"bag_rate_analysis_{mode}.json"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
