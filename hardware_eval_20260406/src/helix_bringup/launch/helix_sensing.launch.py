"""
HELIX Phase 1 bringup launch file.

Starts three lifecycle sensor nodes with parameters from helix_params.yaml.
The rules_file_path for LogParser is set using the installed package share path.

NOTE: Lifecycle nodes start in 'unconfigured' state. Transition them manually:
    ros2 lifecycle set /helix_heartbeat_monitor configure
    ros2 lifecycle set /helix_heartbeat_monitor activate
    # repeat for helix_anomaly_detector and helix_log_parser
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for HELIX Phase 1 fault sensing."""
    bringup_share = get_package_share_directory("helix_bringup")
    params_file = os.path.join(bringup_share, "config", "helix_params.yaml")
    rules_file = os.path.join(bringup_share, "config", "log_rules.yaml")

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

    return LaunchDescription([
        heartbeat_monitor,
        anomaly_detector,
        log_parser,
    ])
