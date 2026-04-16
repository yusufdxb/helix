"""HELIX adapter layer bringup (Phase 1).

Starts the three adapter lifecycle nodes that bridge non-standard robot
topics (GO2 rate streams, JSON state, pose) into /helix/metrics.

NOTE: Lifecycle nodes start in 'unconfigured' state. Transition them:
    ros2 lifecycle set /helix_topic_rate_monitor configure
    ros2 lifecycle set /helix_topic_rate_monitor activate
    # repeat for helix_json_state_parser and helix_pose_drift_monitor
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("helix_bringup")
    params_file = os.path.join(bringup_share, "config", "helix_adapter_params.yaml")

    sim_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description="Remap /utlidar/cloud to /utlidar/cloud_throttled for Isaac Sim",
    )

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

    return LaunchDescription([
        sim_arg,
        topic_rate_monitor,
        json_state_parser,
        pose_drift_monitor,
    ])
