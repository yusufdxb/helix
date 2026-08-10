#!/usr/bin/env python3
"""Validate a HELIX physical-closure bag against explicit pass/fail gates."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path


def _first(items, predicate, after: float | None = None):
    for item in items:
        if after is not None and item["ts"] < after:
            continue
        if predicate(item):
            return item
    return None


def _window(items, start: float, end: float) -> list:
    return [item for item in items if start <= item["ts"] <= end]


def _check(name: str, passed: bool, evidence: dict) -> dict:
    return {"name": name, "passed": bool(passed), "evidence": evidence}


def evaluate(records: dict, max_detection_s: float = 10.0) -> dict:
    checks = []
    drop = _first(records["markers"], lambda item: item["target_hz"] == 0.0)
    checks.append(_check("zero-rate injection marker recorded", drop is not None, {"marker": drop}))
    if drop is None:
        return {"passed": False, "checks": checks, "latencies_s": {}}

    drop_ts = drop["ts"]
    pre_cmd = _window(records["cmd_vel"], drop_ts - 5.0, drop_ts - 0.1)
    moving_cmd = [item for item in pre_cmd if not item["zero"]]
    checks.append(_check(
        "mux commanded nonzero motion before injection",
        bool(moving_cmd),
        {"samples": len(pre_cmd), "nonzero_samples": len(moving_cmd)},
    ))

    fault = _first(
        records["faults"],
        lambda item: (
            item["fault_type"] == "ANOMALY"
            and item["violation_type"] == "stale"
            and "utlidar" in item["metric_name"]
        ),
        after=drop_ts,
    )
    detection_latency = None if fault is None else fault["ts"] - drop_ts
    checks.append(_check(
        "stale LiDAR fault detected within bound",
        fault is not None and 0.0 <= detection_latency <= max_detection_s,
        {"fault": fault, "latency_s": detection_latency, "bound_s": max_detection_s},
    ))
    if fault is None:
        return {"passed": False, "checks": checks, "latencies_s": {}}

    hint = _first(
        records["hints"],
        lambda item: item["action"] == "STOP_AND_HOLD" and item["fault_id"] == fault["fault_id"],
        after=fault["ts"],
    )
    hint_latency = None if hint is None else hint["ts"] - fault["ts"]
    checks.append(_check(
        "STOP_AND_HOLD decision follows the same fault",
        hint is not None and hint_latency <= 0.25,
        {"hint": hint, "latency_s": hint_latency, "bound_s": 0.25},
    ))
    if hint is None:
        return {"passed": False, "checks": checks, "latencies_s": {"detection": detection_latency}}

    action = _first(
        records["actions"],
        lambda item: (
            item["action"] == "STOP_AND_HOLD"
            and item["status"] == "ACCEPTED"
            and item["fault_id"] == hint["fault_id"]
        ),
        after=hint["ts"],
    )
    # The 0.25 s bound only means anything for the FIRST stop of an episode.
    # If a stop was already accepted within the last cooldown_seconds, every
    # later hint is SUPPRESSED_COOLDOWN until the window expires, so this
    # measures time-to-cooldown-expiry, not decision latency. A sim run where
    # HELIX was already holding measured 4.497 s here, and the gaps between
    # accepted actions in that bag were 4.999, 5.005, 4.998, 5.001, 5.005 s:
    # the cooldown, exactly. Read a failure here together with whether the
    # robot was already stopped before injection.
    decision_latency = None if action is None else action["ts"] - hint["ts"]
    checks.append(_check(
        "recovery envelope accepts the same stop decision",
        action is not None and decision_latency <= 0.25,
        {"action": action, "latency_s": decision_latency, "bound_s": 0.25,
         "note": "bound assumes no cooldown was already running"},
    ))
    if action is None:
        return {
            "passed": False,
            "checks": checks,
            "latencies_s": {"detection": detection_latency, "fault_to_hint": hint_latency},
        }

    helix_zero = _first(records["helix_cmd_vel"], lambda item: item["zero"], after=action["ts"])
    mux_zero = _first(records["cmd_vel"], lambda item: item["zero"], after=action["ts"])
    helix_latency = None if helix_zero is None else helix_zero["ts"] - action["ts"]
    mux_latency = None if mux_zero is None else mux_zero["ts"] - action["ts"]
    checks.append(_check(
        "recovery publishes a zero command",
        helix_zero is not None and helix_latency <= 0.25,
        {"latency_s": helix_latency, "bound_s": 0.25},
    ))
    checks.append(_check(
        "mux output carries the zero command",
        mux_zero is not None and mux_latency <= 0.50,
        {"latency_s": mux_latency, "bound_s": 0.50},
    ))

    pre_odom = _window(records["odom"], drop_ts - 3.0, drop_ts - 0.1)
    post_odom = _window(records["odom"], action["ts"] + 1.0, action["ts"] + 3.0)
    pre_speed = statistics.median(item["speed_m_s"] for item in pre_odom) if pre_odom else None
    post_speed = statistics.median(item["speed_m_s"] for item in post_odom) if post_odom else None
    checks.append(_check(
        "odometry proves physical motion before injection",
        pre_speed is not None and pre_speed >= 0.08,
        {"samples": len(pre_odom), "median_speed_m_s": pre_speed, "minimum_m_s": 0.08},
    ))
    checks.append(_check(
        "odometry proves the robot stopped",
        post_speed is not None and post_speed <= 0.03,
        {"samples": len(post_odom), "median_speed_m_s": post_speed, "maximum_m_s": 0.03},
    ))

    latencies = {
        "injection_to_fault": detection_latency,
        "fault_to_hint": hint_latency,
        "hint_to_accept": decision_latency,
        "accept_to_helix_zero": helix_latency,
        "accept_to_mux_zero": mux_latency,
    }
    return {"passed": all(item["passed"] for item in checks), "checks": checks, "latencies_s": latencies}


def _msg_dir() -> Path:
    """Locate helix_msgs/msg, preferring the installed share dir over the source tree."""
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

        try:
            return Path(get_package_share_directory("helix_msgs")) / "msg"
        except PackageNotFoundError:
            pass
    except ImportError:
        pass
    return Path(__file__).resolve().parent.parent / "src" / "helix_msgs" / "msg"


def helix_typestore():
    """Typestore that can decode helix_msgs out of a ROS 2 Humble bag.

    Storage format decides whether this is needed. An MCAP bag embeds its
    schemas and opens on its own. Humble's sqlite3 writer stores no message
    definitions at all, so a reader cannot learn a custom type from the
    recording and fails at open with "Bag contains no type definitions" --
    which is every hardware bag recorded before the scenario runner switched
    to MCAP. Registering the .msg sources keeps those readable, and costs
    nothing for bags that already describe themselves.
    """
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    msg_dir = _msg_dir()
    definitions = sorted(msg_dir.glob("*.msg"))
    if not definitions:
        raise FileNotFoundError(f"no helix_msgs .msg definitions under {msg_dir}")
    for path in definitions:
        typestore.register(
            get_types_from_msg(path.read_text(encoding="utf-8"), f"helix_msgs/msg/{path.stem}")
        )
    return typestore


def load_records(bag_dir: Path) -> dict:
    from rosbags.highlevel import AnyReader

    records = {
        "markers": [],
        "faults": [],
        "hints": [],
        "actions": [],
        "helix_cmd_vel": [],
        "cmd_vel": [],
        "odom": [],
    }
    topics = {
        "/helix/experiment_marker",
        "/helix/faults",
        "/helix/recovery_hints",
        "/helix/recovery_actions",
        "/helix/cmd_vel",
        "/cmd_vel",
        "/utlidar/robot_odom",
    }
    with AnyReader([bag_dir], default_typestore=helix_typestore()) as reader:
        connections = [connection for connection in reader.connections if connection.topic in topics]
        for connection, timestamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            ts = timestamp / 1e9
            if connection.topic == "/helix/experiment_marker":
                marker = json.loads(message.data)
                records["markers"].append({"ts": ts, **marker})
            elif connection.topic == "/helix/faults":
                context = dict(zip(message.context_keys, message.context_values))
                records["faults"].append({
                    "ts": ts,
                    "fault_id": message.node_name,
                    "fault_type": message.fault_type,
                    "metric_name": context.get("metric_name", ""),
                    "violation_type": context.get("violation_type", ""),
                })
            elif connection.topic == "/helix/recovery_hints":
                records["hints"].append({
                    "ts": ts,
                    "fault_id": message.fault_id,
                    "action": message.suggested_action,
                })
            elif connection.topic == "/helix/recovery_actions":
                records["actions"].append({
                    "ts": ts,
                    "fault_id": message.fault_id,
                    "action": message.action,
                    "status": message.status,
                })
            elif connection.topic in {"/helix/cmd_vel", "/cmd_vel"}:
                values = [
                    message.linear.x, message.linear.y, message.linear.z,
                    message.angular.x, message.angular.y, message.angular.z,
                ]
                key = "helix_cmd_vel" if connection.topic == "/helix/cmd_vel" else "cmd_vel"
                records[key].append({"ts": ts, "zero": all(abs(value) <= 1e-4 for value in values)})
            elif connection.topic == "/utlidar/robot_odom":
                linear = message.twist.twist.linear
                speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
                records["odom"].append({"ts": ts, "speed_m_s": speed})
    return records


def evidence_files(bag_dir: Path) -> list[dict]:
    artifacts = []
    for path in sorted(item for item in bag_dir.rglob("*") if item.is_file()):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        artifacts.append({
            "path": str(path.relative_to(bag_dir)),
            "bytes": path.stat().st_size,
            "sha256": digest.hexdigest(),
        })
    return artifacts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-detection-s", type=float, default=10.0)
    args = parser.parse_args()
    result = evaluate(load_records(args.bag), max_detection_s=args.max_detection_s)
    result.update({"schema_version": 1, "bag": str(args.bag), "artifacts": evidence_files(args.bag)})
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
