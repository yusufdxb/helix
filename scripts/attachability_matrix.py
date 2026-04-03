#!/usr/bin/env python3
"""
Compute HELIX Attachability Matrix for a target ROS 2 platform.

"Attachability" measures how much of a monitoring architecture's input
interface is natively satisfied by a target platform, and what adapter
effort is required to bridge the gap.

Connects to a live ROS 2 graph (or reads from a saved topic list file)
and evaluates:
  - native input coverage (what HELIX can use as-is)
  - adaptable coverage (what can be bridged with standard-type adapters)
  - unreachable coverage (requires custom message packages or SDK integration)

Usage:
  source /opt/ros/humble/setup.bash
  python3 scripts/attachability_matrix.py [--from-file topics.txt]
"""

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"

# ── HELIX Input Specification ────────────────────────────────────────────────

HELIX_INPUTS = {
    "/diagnostics": {
        "type": "diagnostic_msgs/msg/DiagnosticArray",
        "node": "anomaly_detector",
        "purpose": "Numeric KV pairs for Z-score anomaly detection",
        "required": False,
    },
    "/helix/metrics": {
        "type": "std_msgs/msg/Float64MultiArray",
        "node": "anomaly_detector",
        "purpose": "Labeled numeric metric streams",
        "required": True,
    },
    "/helix/heartbeat": {
        "type": "std_msgs/msg/String",
        "node": "heartbeat_monitor",
        "purpose": "Node liveness heartbeats",
        "required": True,
    },
    "/rosout": {
        "type": "rcl_interfaces/msg/Log",
        "node": "log_parser",
        "purpose": "Log messages for regex pattern matching",
        "required": True,
    },
}

# ── Standard ROS 2 Types (no custom packages needed) ────────────────────────

STANDARD_TYPES = {
    "std_msgs/", "sensor_msgs/", "geometry_msgs/", "nav_msgs/",
    "diagnostic_msgs/", "rcl_interfaces/", "tf2_msgs/",
    "visualization_msgs/", "shape_msgs/", "trajectory_msgs/",
    "action_msgs/", "builtin_interfaces/", "unique_identifier_msgs/",
}

# Topics matching these types have a demonstrated or strongly motivated adapter
# path to HELIX /helix/metrics.  Topics that are merely std_msgs/msg/String
# are *candidates* — the content may or may not be JSON-parseable with numeric
# fields.  We label them separately so callers can distinguish demonstrated
# adapters from heuristic candidates.
DEMONSTRATED_ADAPTER_TYPES = {
    "sensor_msgs/msg/Imu": "imu_rate_monitor",
    "nav_msgs/msg/Odometry": "odom_drift_monitor",
    "geometry_msgs/msg/PoseStamped": "pose_drift_monitor",
    "sensor_msgs/msg/PointCloud2": "pointcloud_rate_monitor",
    "sensor_msgs/msg/JointState": "joint_state_monitor",
}

# std_msgs/msg/String topics are heuristic candidates: they *might* carry
# JSON with numeric fields, but we have no compile-time guarantee.
CANDIDATE_ADAPTER_TYPE = "std_msgs/msg/String"


def is_standard_type(msg_type: str) -> bool:
    return any(msg_type.startswith(prefix) for prefix in STANDARD_TYPES)


def get_live_topics() -> list:
    """Get topic list from live ROS 2 graph."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "list", "-t"],
            capture_output=True, text=True, timeout=10
        )
        topics = []
        for line in result.stdout.strip().split("\n"):
            if " [" in line and "]" in line:
                topic = line.split(" [")[0].strip()
                msg_type = line.split("[")[1].rstrip("]").strip()
                topics.append({"topic": topic, "type": msg_type})
        return topics
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []


def compute_attachability(topics: list) -> dict:
    """Compute the attachability matrix."""

    # Classify each platform topic.
    # Duplicate topic names are deduplicated (first occurrence wins).
    platform_topics = {}
    for t in topics:
        topic = t["topic"]
        if topic in platform_topics:
            continue  # keep first occurrence
        msg_type = t["type"]
        standard = is_standard_type(msg_type)
        platform_topics[topic] = {
            "type": msg_type,
            "is_standard": standard,
        }

    total_topics = len(platform_topics)
    standard_count = sum(1 for t in platform_topics.values() if t["is_standard"])
    custom_count = total_topics - standard_count

    # Check HELIX input coverage
    input_coverage = {}
    for helix_topic, spec in HELIX_INPUTS.items():
        if helix_topic in platform_topics:
            pt = platform_topics[helix_topic]
            if pt["type"] == spec["type"]:
                input_coverage[helix_topic] = "native"
            else:
                input_coverage[helix_topic] = "type_mismatch"
        else:
            input_coverage[helix_topic] = "missing"

    native_count = sum(1 for v in input_coverage.values() if v == "native")

    # Identify adaptable topics (standard types that could feed HELIX).
    # We split into two tiers:
    #   demonstrated — type has a known, tested adapter path
    #   candidate   — std_msgs/msg/String that *might* contain parseable JSON
    demonstrated = []
    candidates = []
    for topic, info in platform_topics.items():
        if not info["is_standard"] or topic in HELIX_INPUTS:
            continue
        msg_type = info["type"]
        if msg_type in DEMONSTRATED_ADAPTER_TYPES:
            demonstrated.append({
                "topic": topic,
                "type": msg_type,
                "adapter": DEMONSTRATED_ADAPTER_TYPES[msg_type],
                "effort": "low",
                "confidence": "demonstrated",
            })
        elif msg_type == CANDIDATE_ADAPTER_TYPE:
            candidates.append({
                "topic": topic,
                "type": msg_type,
                "adapter": "json_state_parser",
                "effort": "low",
                "confidence": "candidate",
            })

    adaptable = demonstrated + candidates

    # Custom-type topics that need packages built
    unreachable = []
    for topic, info in platform_topics.items():
        if not info["is_standard"]:
            unreachable.append({
                "topic": topic,
                "type": info["type"],
                "barrier": "custom_message_package",
                "effort": "high",
            })

    # Compute scores
    #   native_coverage : fraction of HELIX inputs satisfied by the platform as-is
    #   with_adapters   : fraction of HELIX inputs satisfiable if all adaptable
    #                     topics were bridged (capped at 1.0)
    #   adaptable_topics_count : how many platform topics have a plausible adapter
    n_inputs = len(HELIX_INPUTS)
    gaps = n_inputs - native_count                       # inputs still missing
    fillable = min(len(adaptable), gaps)                 # adapters can fill at most the gaps
    native_score = native_count / n_inputs
    with_adapters_score = (native_count + fillable) / n_inputs  # always <= 1.0

    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "platform_summary": {
            "total_topics": total_topics,
            "standard_type_topics": standard_count,
            "custom_type_topics": custom_count,
            "standard_fraction": round(standard_count / max(1, total_topics), 3),
        },
        "helix_input_coverage": input_coverage,
        "scores": {
            "native_coverage": round(native_score, 3),
            "with_adapters": round(with_adapters_score, 3),
            "adaptable_topics_count": len(adaptable),
            "demonstrated_adapter_count": len(demonstrated),
            "candidate_adapter_count": len(candidates),
            "unreachable_topics_count": len(unreachable),
        },
        "adaptable_topics": adaptable[:20],
        "unreachable_topics_sample": unreachable[:10],
        "interpretation": {
            "native": f"{native_count}/{n_inputs} HELIX inputs available natively",
            "adapter_path": (
                f"{len(demonstrated)} topics have demonstrated adapters, "
                f"{len(candidates)} are heuristic candidates (std_msgs/String)"
            ),
            "custom_barrier": f"{len(unreachable)} topics behind custom message type barrier",
        },
    }


def main():
    parser = argparse.ArgumentParser(description="HELIX Attachability Matrix")
    parser.add_argument("--from-file", help="Read topic list from file instead of live graph")
    args = parser.parse_args()

    if args.from_file:
        # Parse saved topic list (format: topic [type])
        topics = []
        with open(args.from_file) as f:
            for line in f:
                line = line.strip()
                if " [" in line and "]" in line:
                    topic = line.split(" [")[0].strip()
                    msg_type = line.split("[")[1].rstrip("]").strip()
                    topics.append({"topic": topic, "type": msg_type})
        print(f"Loaded {len(topics)} topics from {args.from_file}")
    else:
        topics = get_live_topics()
        print(f"Discovered {len(topics)} topics from live ROS 2 graph")

    if not topics:
        print("ERROR: No topics found", file=sys.stderr)
        sys.exit(1)

    results = compute_attachability(topics)

    # Print summary
    print(f"\n{'='*60}")
    print(f"  HELIX Attachability Matrix")
    print(f"{'='*60}")
    print(f"  Platform topics:    {results['platform_summary']['total_topics']}")
    print(f"  Standard types:     {results['platform_summary']['standard_type_topics']} "
          f"({results['platform_summary']['standard_fraction']:.0%})")
    print(f"  Custom types:       {results['platform_summary']['custom_type_topics']}")
    print()
    print(f"  HELIX Input Coverage:")
    for topic, status in results["helix_input_coverage"].items():
        symbol = {"native": "+", "missing": "-", "type_mismatch": "~"}[status]
        print(f"    [{symbol}] {topic}: {status}")
    print()
    print(f"  Native score:       {results['scores']['native_coverage']:.0%}")
    print(f"  With adapters:      {results['scores']['with_adapters']:.0%}")
    print(f"  Adaptable topics:   {results['scores']['adaptable_topics_count']}")
    print(f"  Unreachable:        {results['scores']['unreachable_topics_count']}")
    print(f"{'='*60}")

    # Save
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = RESULTS_DIR / "attachability_matrix.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
