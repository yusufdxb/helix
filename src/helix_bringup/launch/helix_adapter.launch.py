"""HELIX adapter-layer bringup.

Starts the three lifecycle adapter nodes from ``helix_adapter`` that bridge
non-standard robot topics (GO2 rate streams, JSON state, pose) into
``/helix/metrics``. Auto-transitions them through configure -> active by
default so the documented path produces a running adapter stack.

Pass ``auto_activate:=false`` to drive the lifecycle by hand:
    ros2 launch helix_bringup helix_adapter.launch.py auto_activate:=false
    ros2 lifecycle set /helix_topic_rate_monitor configure
    ros2 lifecycle set /helix_topic_rate_monitor activate
    # repeat for helix_json_state_parser and helix_pose_drift_monitor
"""
import os

import launch.events
from ament_index_python.packages import get_package_share_directory
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

from launch import LaunchDescription


def _auto_activate(node: LifecycleNode, condition):
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
    params_file = os.path.join(bringup_share, "config", "helix_adapter_params.yaml")

    sim_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description="Remap /utlidar/cloud to /utlidar/cloud_throttled for Isaac Sim",
    )
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

    topic_rate_monitor = LifecycleNode(
        package="helix_adapter",
        executable="helix_topic_rate_monitor",
        name="helix_topic_rate_monitor",
        namespace="",
        parameters=[
            params_file,
            {"sim_mode": LaunchConfiguration("sim_mode")},
        ],
        output="screen",
    )

    json_state_parser = LifecycleNode(
        package="helix_adapter",
        executable="helix_json_state_parser",
        name="helix_json_state_parser",
        namespace="",
        parameters=[params_file],
        output="screen",
    )

    pose_drift_monitor = LifecycleNode(
        package="helix_adapter",
        executable="helix_pose_drift_monitor",
        name="helix_pose_drift_monitor",
        namespace="",
        parameters=[params_file],
        output="screen",
    )

    actions = [
        sim_arg,
        auto_activate_arg,
        topic_rate_monitor,
        json_state_parser,
        pose_drift_monitor,
    ]
    for node in (topic_rate_monitor, json_state_parser, pose_drift_monitor):
        actions.extend(_auto_activate(node, auto_activate_cond))
    return LaunchDescription(actions)
