"""HELIX Phase 1 sensing-stack bringup.

Starts the three lifecycle sensor nodes from ``helix_core`` and (by default)
auto-transitions them through configure -> activate so the documented quick
start produces an actually-running stack — not three nodes parked in the
``unconfigured`` state.

Pass ``auto_activate:=false`` if you want to drive the lifecycle by hand:
    ros2 launch helix_bringup helix_sensing.launch.py auto_activate:=false
    ros2 lifecycle set /helix_heartbeat_monitor configure
    ros2 lifecycle set /helix_heartbeat_monitor activate
    # repeat for helix_anomaly_detector and helix_log_parser
"""
import os

import launch.events
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _auto_activate(node: LifecycleNode, condition):
    """Emit configure on launch, then activate when the node reaches 'inactive'."""
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=condition,
    )
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[helix_bringup] auto-activating lifecycle node"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=condition,
    )
    return [configure, activate_on_inactive]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("helix_bringup")
    params_file = os.path.join(bringup_share, "config", "helix_params.yaml")
    rules_file = os.path.join(bringup_share, "config", "log_rules.yaml")

    auto_activate_arg = DeclareLaunchArgument(
        "auto_activate",
        default_value="true",
        description=(
            "If true (default), the launch transitions all lifecycle nodes "
            "through configure -> active. Set to false to drive the lifecycle "
            "manually via 'ros2 lifecycle set ...'."
        ),
    )
    auto_activate_cond = IfCondition(LaunchConfiguration("auto_activate"))

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

    actions = [auto_activate_arg, heartbeat_monitor, anomaly_detector, log_parser]
    for node in (heartbeat_monitor, anomaly_detector, log_parser):
        actions.extend(_auto_activate(node, auto_activate_cond))
    return LaunchDescription(actions)
