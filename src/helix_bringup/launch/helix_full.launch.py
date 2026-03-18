"""
HELIX full bringup — Phase 1 (sensing) + Phase 2 (recovery).

All four nodes are launched as LifecycleNodes.
TimerActions fire at t=2s (configure) and t=4s (activate) using ExecuteProcess
to call `ros2 lifecycle set` — more reliable than the ChangeState event-handler API.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    """Generate the HELIX full launch description (Phase 1 + Phase 2)."""
    bringup_share = get_package_share_directory("helix_bringup")
    params_file = os.path.join(bringup_share, "config", "helix_params.yaml")
    rules_file = os.path.join(bringup_share, "config", "log_rules.yaml")
    policy_file = os.path.join(bringup_share, "config", "recovery_policy.yaml")

    # ── Sensing nodes (Phase 1) ──────────────────────────────────────────────
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

    # ── Recovery node (Phase 2) ──────────────────────────────────────────────
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

    # ── Auto-transition all nodes to active ─────────────────────────────────
    # Use ExecuteProcess (ros2 lifecycle set) — avoids the fragile ChangeState
    # event-handler API which requires matching internal launch framework events.
    all_node_names = [
        "helix_heartbeat_monitor",
        "helix_anomaly_detector",
        "helix_log_parser",
        "helix_recovery_planner",
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

    # Fire configure at t=2s, activate at t=4s
    auto_configure = TimerAction(period=2.0, actions=configure_cmds)
    auto_activate = TimerAction(period=4.0, actions=activate_cmds)

    return LaunchDescription([
        heartbeat_monitor,
        anomaly_detector,
        log_parser,
        recovery_planner,
        auto_configure,
        auto_activate,
    ])
