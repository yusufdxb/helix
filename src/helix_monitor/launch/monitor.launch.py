"""Start the observe-only HELIX monitor. It subscribes to GO2 topics and never publishes commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='', description='monitor config; empty = share/helix_monitor/config/go2.cfg'),
        Node(package='helix_monitor', executable='helix_monitor_node', name='helix_monitor', output='screen',
             parameters=[{'config': LaunchConfiguration('config')}]),
    ])
