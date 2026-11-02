"""Launch the HELIX healer on a spec file (defaults to the bundled nav example)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_spec = os.path.join(get_package_share_directory("helix_healer"), "config", "nav.spec")
    return LaunchDescription([
        DeclareLaunchArgument("spec", default_value=default_spec),
        DeclareLaunchArgument("policy", default_value="helix"),
        DeclareLaunchArgument("event_log", default_value=""),
        DeclareLaunchArgument("log_dir", default_value=""),
        Node(
            package="helix_healer",
            executable="helix_healer_node",
            name="helix_healer",
            output="screen",
            parameters=[{
                "spec": LaunchConfiguration("spec"),
                "policy": LaunchConfiguration("policy"),
                "event_log": LaunchConfiguration("event_log"),
                "log_dir": LaunchConfiguration("log_dir"),
            }],
        ),
    ])
