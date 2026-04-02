#!/usr/bin/env python3
"""GO2 topic gap analysis for HELIX.

Compares the ROS 2 topics published by a Unitree GO2 robot against
the topics HELIX subscribes to, and prints a categorized summary.

Data source: Live capture from GO2 (192.168.123.161) observed via
Jetson Orin NX (192.168.123.18) on 2026-04-02 using FastRTPS.
See docs/GO2_TOPIC_ANALYSIS.md for full analysis.
"""

from __future__ import annotations

import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Embedded GO2 topic list — captured 2026-04-02 from ros2_topic_list.txt
# Default external path points to T7 capture drive; falls back to this list.
# ---------------------------------------------------------------------------
DEFAULT_TOPIC_FILE = (
    "/media/yusuf/T7 Storage/go2_data_capture_20260402/ros2_topic_list.txt"
)

GO2_TOPICS_RAW = """\
/api/assistant_recorder/request
/api/assistant_recorder/response
/api/audiohub/request
/api/audiohub/response
/api/bashrunner/request
/api/bashrunner/response
/api/config/request
/api/config/response
/api/fourg_agent/request
/api/fourg_agent/response
/api/gas_sensor/request
/api/gas_sensor/response
/api/gesture/request
/api/gpt/request
/api/gpt/response
/api/motion_switcher/request
/api/motion_switcher/response
/api/obstacles_avoid/request
/api/obstacles_avoid/response
/api/pet/request
/api/pet/response
/api/programming_actuator/request
/api/programming_actuator/response
/api/rm_con/request
/api/robot_state/request
/api/robot_state/response
/api/sport/request
/api/sport/response
/api/sport_lease/request
/api/sport_lease/response
/api/uwbswitch/request
/api/uwbswitch/response
/api/videohub/request
/api/videohub/response
/api/vui/request
/api/vui/response
/arm_Command
/arm_Feedback
/audio/raw
/audiohub/player/state
/audioreceiver
/audiosender
/camera/camera_info
/camera/image_raw
/cmd_vel_out
/config_change_status
/frontvideostream
/gas_sensor
/gesture/result
/gnss
/go2_states
/gpt_cmd
/gptflowfeedback
/imu
/joint_states
/joy
/lf/battery_alarm
/lf/lowstate
/lf/sportmodestate
/lio_sam_ros2/mapping/odometry
/lowcmd
/lowstate
/multiplestate
/odom
/parameter_events
/pctoimage_local
/pet/flowfeedback
/point_cloud2
/programming_actuator/command
/programming_actuator/feedback
/public_network_status
/qt_add_edge
/qt_add_node
/qt_command
/qt_notice
/query_result_edge
/query_result_node
/rosout
/rtc/state
/rtc_status
/selftest
/servicestate
/servicestateactivate
/sportmodestate
/tf
/uslam/client_command
/uslam/cloud_map
/uslam/frontend/cloud_world_ds
/uslam/frontend/odom
/uslam/localization/cloud_world
/uslam/localization/odom
/uslam/navigation/global_path
/uslam/server_log
/utlidar/client_cmd
/utlidar/cloud
/utlidar/cloud_base
/utlidar/cloud_deskewed
/utlidar/grid_map
/utlidar/height_map
/utlidar/height_map_array
/utlidar/imu
/utlidar/lidar_state
/utlidar/mapping_cmd
/utlidar/range_info
/utlidar/range_map
/utlidar/robot_odom
/utlidar/robot_pose
/utlidar/server_log
/utlidar/switch
/utlidar/voxel_map
/utlidar/voxel_map_compressed
/uwbstate
/uwbswitch
/videohub/inner
/webrtc_req
/webrtcreq
/webrtcres
/wirelesscontroller
/wirelesscontroller_unprocessed
/xfk_webrtcreq
/xfk_webrtcres
"""


# ---------------------------------------------------------------------------
# Category definitions
# ---------------------------------------------------------------------------
CATEGORIES: dict[str, list[str]] = {
    "Robot State/Control": [
        "/sportmodestate", "/lowstate", "/lowcmd", "/lf/lowstate",
        "/lf/sportmodestate", "/imu", "/joint_states", "/odom",
        "/go2_states", "/multiplestate", "/cmd_vel_out", "/joy",
        "/arm_Command", "/arm_Feedback", "/selftest", "/servicestate",
        "/servicestateactivate", "/wirelesscontroller",
        "/wirelesscontroller_unprocessed",
    ],
    "Perception": [
        "/camera/camera_info", "/camera/image_raw", "/frontvideostream",
        "/point_cloud2",
    ] + [t for t in GO2_TOPICS_RAW.strip().splitlines() if t.startswith("/utlidar/")],
    "Navigation/SLAM": [
        t for t in GO2_TOPICS_RAW.strip().splitlines() if t.startswith("/uslam/")
    ] + ["/lio_sam_ros2/mapping/odometry", "/pctoimage_local"],
    "API Services": [
        t for t in GO2_TOPICS_RAW.strip().splitlines() if t.startswith("/api/")
    ],
    "Communication/Networking": [
        "/audio/raw", "/audiohub/player/state", "/audioreceiver",
        "/audiosender", "/webrtc_req", "/webrtcreq", "/webrtcres",
        "/xfk_webrtcreq", "/xfk_webrtcres", "/public_network_status",
        "/rtc/state", "/rtc_status", "/videohub/inner",
    ],
    "System": [
        "/rosout", "/parameter_events", "/tf", "/config_change_status",
    ],
}

# Topics HELIX subscribes to, with expected message types.
HELIX_REQUIRED = {
    "/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
    "/helix/heartbeat": "std_msgs/msg/String",
    "/helix/metrics": "std_msgs/msg/Float64MultiArray",
    "/rosout": "rcl_interfaces/msg/Log",
}


def load_topics(path: str | None = None) -> list[str]:
    """Load topics from file or fall back to embedded list."""
    if path:
        p = Path(path)
        if p.is_file():
            print(f"[info] Loading topics from: {p}")
            return [
                line.strip()
                for line in p.read_text().splitlines()
                if line.strip()
            ]
    # Try default external path
    default = Path(DEFAULT_TOPIC_FILE)
    if default.is_file():
        print(f"[info] Loading topics from: {default}")
        return [
            line.strip()
            for line in default.read_text().splitlines()
            if line.strip()
        ]
    print("[info] Using embedded topic list (captured 2026-04-02)")
    return [line.strip() for line in GO2_TOPICS_RAW.strip().splitlines() if line.strip()]


def categorize(topics: list[str]) -> dict[str, list[str]]:
    """Assign each topic to a category."""
    assigned: set[str] = set()
    result: dict[str, list[str]] = {}
    for cat, members in CATEGORIES.items():
        matched = [t for t in topics if t in members]
        result[cat] = sorted(set(matched))
        assigned.update(matched)
    uncategorized = sorted(set(topics) - assigned)
    if uncategorized:
        result["Other"] = uncategorized
    return result


def print_summary(topics: list[str], cats: dict[str, list[str]]) -> None:
    """Print the gap analysis summary."""
    topic_set = set(topics)

    print("=" * 65)
    print("  GO2 Topic Gap Analysis for HELIX")
    print("  Data: live capture 2026-04-02 | GO2 192.168.123.161")
    print("=" * 65)
    print()

    # Category summary
    print(f"Total GO2 topics: {len(topics)}")
    print()
    print("Category breakdown:")
    print("-" * 45)
    for cat, members in cats.items():
        print(f"  {cat:<30s} {len(members):>3d} topics")
    print("-" * 45)
    total_categorized = sum(len(m) for m in cats.values())
    print(f"  {'Total':<30s} {total_categorized:>3d} topics")
    print()

    # HELIX gap analysis
    print("HELIX required topics:")
    print("-" * 65)
    available = 0
    for topic, msg_type in HELIX_REQUIRED.items():
        present = topic in topic_set
        status = "AVAILABLE" if present else "MISSING"
        marker = "  [OK]  " if present else "  [GAP] "
        print(f"{marker} {topic:<30s} {msg_type}")
        if present:
            available += 1
    print("-" * 65)
    print(
        f"\nResult: {available}/{len(HELIX_REQUIRED)} HELIX input channels "
        f"have a data source on the GO2."
    )
    print()

    # Actionable summary
    missing = [t for t in HELIX_REQUIRED if t not in topic_set]
    if missing:
        print("Required integration work:")
        for topic in missing:
            if topic == "/diagnostics":
                print(f"  - {topic}: no diagnostic publisher on GO2; "
                      "need adapter or standard diagnostics aggregator")
            elif topic == "/helix/heartbeat":
                print(f"  - {topic}: need heartbeat bridge nodes for "
                      "each monitored GO2 process")
            elif topic == "/helix/metrics":
                print(f"  - {topic}: need adapter to extract numeric "
                      "values from GO2 state topics")
        print()

    # Custom message warning
    print("Custom message types (require Unitree packages to use):")
    custom_types = {
        "/sportmodestate": "unitree_go/msg/SportModeState",
        "/lowstate": "unitree_go/msg/LowState",
        "/imu": "go2_interfaces/msg/IMU",
    }
    for topic, msg_type in custom_types.items():
        if topic in topic_set:
            print(f"  - {topic}: {msg_type}")
    print()
    print("Conclusion: 1 of 4 HELIX inputs works out of the box.")
    print("The gap is bridgeable — not an architectural incompatibility.")


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else None
    topics = load_topics(path)
    cats = categorize(topics)
    print_summary(topics, cats)


if __name__ == "__main__":
    main()
