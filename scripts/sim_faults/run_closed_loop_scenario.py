#!/usr/bin/env python3
"""Run a marked HELIX closed-loop scenario and capture an auditable ROS bag."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path


RECORDED_TOPICS = [
    "/helix/experiment_marker",
    "/helix/faults",
    "/helix/recovery_hints",
    "/helix/recovery_actions",
    "/helix/cmd_vel",
    "/cmd_vel",
    "/nav/cmd_vel",
    "/utlidar/cloud_throttled",
    "/utlidar/robot_odom",
]


def default_storage() -> str:
    """Prefer self-describing MCAP, fall back to sqlite3 where it is absent.

    Not every deployment has ros-humble-rosbag2-storage-mcap installed (the
    Jetson image notably may not), and silently failing to record is worse
    than recording a bag that needs a typestore to read.
    """
    try:
        from ament_index_python.packages import get_packages_with_prefixes

        if "rosbag2_storage_mcap" in get_packages_with_prefixes():
            return "mcap"
    except ImportError:
        pass
    return "sqlite3"


def build_commands(args, artifact_dir: Path) -> dict[str, list[str]]:
    return {
        "launch": [
            "ros2", "launch", "helix_bringup", "helix_closedloop.launch.py",
            "sim_mode:=true",
            "enable_twist_mux:=true",
            "auto_activate_recovery:=true",
            "recovery_enabled:=true",
            "cmd_vel_out:=/cmd_vel",
        ],
        "record": [
            "ros2", "bag", "record", "-s", args.storage,
            "-o", str(artifact_dir / "bag"),
            *RECORDED_TOPICS,
        ],
        "inject": [
            "python3", "scripts/sim_faults/inject_lidar_rate_drop.py",
            "--schedule", args.schedule,
        ],
        "navigate": [
            "ros2", "topic", "pub", "--rate", "10", "/nav/cmd_vel",
            "geometry_msgs/msg/Twist",
            f"{{linear: {{x: {args.nav_speed}}}}}",
        ],
    }


def _start(command: list[str]) -> subprocess.Popen:
    return subprocess.Popen(command, preexec_fn=os.setsid)


def _stop(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=80.0)
    parser.add_argument("--schedule", default="20:10,20:0,30:10")
    parser.add_argument("--nav-speed", type=float, default=0.20)
    parser.add_argument("--artifact-dir", default=f"results/sim_run_{int(time.time())}")
    parser.add_argument(
        "--storage", default=default_storage(), choices=["mcap", "sqlite3"],
        help="Bag storage plugin. mcap embeds the message definitions in the "
             "file, so an evidence bag stays readable by any reader without "
             "the source tree that produced it; Humble's sqlite3 writer stores "
             "no definitions at all and custom types can only be decoded by a "
             "reader that already knows them.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the exact process plan without starting ROS or writing artifacts.",
    )
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if not 0 < args.nav_speed <= 0.30:
        parser.error("--nav-speed must be in (0, 0.30] m/s")

    artifact_dir = Path(args.artifact_dir)
    commands = build_commands(args, artifact_dir)
    plan = {
        "schema_version": 1,
        "mode": "simulation",
        "duration_s": args.duration,
        "schedule": args.schedule,
        "nav_speed_m_s": args.nav_speed,
        "storage": args.storage,
        "recorded_topics": RECORDED_TOPICS,
        "commands": commands,
    }
    if args.dry_run:
        print(json.dumps(plan, indent=2, sort_keys=True))
        return 0

    if artifact_dir.exists() and any(artifact_dir.iterdir()):
        raise FileExistsError(f"artifact directory is not empty: {artifact_dir}")
    artifact_dir.mkdir(parents=True, exist_ok=True)
    (artifact_dir / "run_metadata.json").write_text(
        json.dumps(plan, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    processes: list[subprocess.Popen] = []
    try:
        print("[scenario] launching canonical HELIX closed loop")
        launch = _start(commands["launch"])
        processes.append(launch)
        time.sleep(8.0)
        if launch.poll() is not None:
            raise RuntimeError(f"HELIX launch exited early with code {launch.returncode}")

        print(f"[scenario] recording marked evidence to {artifact_dir / 'bag'}")
        processes.append(_start(commands["record"]))
        time.sleep(2.0)
        processes.append(_start(commands["navigate"]))
        processes.append(_start(commands["inject"]))

        print(f"[scenario] running for {args.duration:.1f}s")
        time.sleep(args.duration)
    finally:
        print("[scenario] terminating child processes")
        for process in reversed(processes):
            _stop(process)

    print(f"[scenario] artifacts at {artifact_dir}")
    print(
        "[scenario] validate with: python3 scripts/validate_closed_loop_bag.py "
        f"{artifact_dir / 'bag'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
