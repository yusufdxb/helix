"""
HELIX full bringup with LLM advisor — Phase 1 + Phase 2 + Phase 3.

Extends helix_full.launch.py by adding the helix_llm_advisor LifecycleNode.
All five nodes are configured and activated via TimerAction + ExecuteProcess.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    """Generate HELIX full launch description with LLM advisor (Phase 1+2+3)."""
    bringup_share = get_package_share_directory("helix_bringup")
    params_file = os.path.join(bringup_share, "config", "helix_params.yaml")
    rules_file = os.path.join(bringup_share, "config", "log_rules.yaml")
    policy_file = os.path.join(bringup_share, "config", "recovery_policy.yaml")

    # ── Phase 1: Sensing nodes ───────────────────────────────────────────────
    heartbeat_monitor = LifecycleNode(
        package="helix_core",
        executable="helix_heartbeat_monitor",
        name="helix_heartbeat_monitor",
        namespace="",
        parameters=[params_file],
        output="screen",
    )

    anomaly_detector = LifecycleNode(
        package="helix_core",
        executable="helix_anomaly_detector",
        name="helix_anomaly_detector",
        namespace="",
        parameters=[params_file],
        output="screen",
    )

    log_parser = LifecycleNode(
        package="helix_core",
        executable="helix_log_parser",
        name="helix_log_parser",
        namespace="",
        parameters=[
            params_file,
            {"rules_file_path": rules_file},
        ],
        output="screen",
    )

    # ── Phase 2: Recovery node ───────────────────────────────────────────────
    recovery_planner = LifecycleNode(
        package="helix_recovery",
        executable="helix_recovery_planner",
        name="helix_recovery_planner",
        namespace="",
        parameters=[
            params_file,
            {"policy_file_path": policy_file},
        ],
        output="screen",
    )

    # ── Phase 3: LLM advisor node ────────────────────────────────────────────
    llm_advisor = LifecycleNode(
        package="helix_llm",
        executable="helix_llm_advisor",
        name="helix_llm_advisor",
        namespace="",
        parameters=[params_file],
        output="screen",
    )

    # ── Auto-transition all nodes to active ──────────────────────────────────
    all_node_names = [
        "helix_heartbeat_monitor",
        "helix_anomaly_detector",
        "helix_log_parser",
        "helix_recovery_planner",
        "helix_llm_advisor",
    ]

    configure_cmds = [
        ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", f"/{name}", "configure"],
            output="screen",
        )
        for name in all_node_names
    ]

    activate_cmds = [
        ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", f"/{name}", "activate"],
            output="screen",
        )
        for name in all_node_names
    ]

    auto_configure = TimerAction(period=2.0, actions=configure_cmds)
    auto_activate = TimerAction(period=4.0, actions=activate_cmds)

    return LaunchDescription([
        heartbeat_monitor,
        anomaly_detector,
        log_parser,
        recovery_planner,
        llm_advisor,
        auto_configure,
        auto_activate,
    ])
