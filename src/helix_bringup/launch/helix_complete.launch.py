"""
helix_complete.launch.py — HELIX definitive bringup (all phases).

Launches all 6 nodes:
  Phase 1: helix_heartbeat_monitor, helix_anomaly_detector, helix_log_parser
  Phase 2: helix_recovery_planner
  Phase 3: helix_llm_advisor
  Phase 4: helix_dashboard_node (plain node — always on, no lifecycle)

Lifecycle nodes are configured at t=2s and activated at t=4s.
Dashboard node starts immediately (no lifecycle).

After launch, dashboard is available at http://localhost:8080.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description() -> LaunchDescription:
    """Generate the complete HELIX launch description (Phases 1–4)."""
    bringup_share = get_package_share_directory("helix_bringup")
    params_file = os.path.join(bringup_share, "config", "helix_params.yaml")
    rules_file = os.path.join(bringup_share, "config", "log_rules.yaml")
    policy_file = os.path.join(bringup_share, "config", "recovery_policy.yaml")

    # ── Phase 1: Sensing ─────────────────────────────────────────────────────
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
        parameters=[params_file, {"rules_file_path": rules_file}],
        output="screen",
    )

    # ── Phase 2: Recovery ────────────────────────────────────────────────────
    recovery_planner = LifecycleNode(
        package="helix_recovery",
        executable="helix_recovery_planner",
        name="helix_recovery_planner",
        namespace="",
        parameters=[params_file, {"policy_file_path": policy_file}],
        output="screen",
    )

    # ── Phase 3: LLM Advisor ─────────────────────────────────────────────────
    llm_advisor = LifecycleNode(
        package="helix_llm",
        executable="helix_llm_advisor",
        name="helix_llm_advisor",
        namespace="",
        parameters=[params_file],
        output="screen",
    )

    # ── Phase 4: Dashboard (plain Node — always on) ──────────────────────────
    dashboard = Node(
        package="helix_dashboard",
        executable="helix_dashboard_node",
        name="helix_dashboard_node",
        namespace="",
        parameters=[params_file],
        output="screen",
    )

    # ── Auto-transition lifecycle nodes ──────────────────────────────────────
    lifecycle_names = [
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
        for name in lifecycle_names
    ]
    activate_cmds = [
        ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", f"/{name}", "activate"],
            output="screen",
        )
        for name in lifecycle_names
    ]

    auto_configure = TimerAction(period=5.0, actions=configure_cmds)
    auto_activate = TimerAction(period=8.0, actions=activate_cmds)

    dashboard_ready = TimerAction(
        period=10.0,
        actions=[LogInfo(msg="HELIX dashboard available at http://localhost:8080")],
    )

    return LaunchDescription([
        heartbeat_monitor,
        anomaly_detector,
        log_parser,
        recovery_planner,
        llm_advisor,
        dashboard,
        auto_configure,
        auto_activate,
        dashboard_ready,
    ])
