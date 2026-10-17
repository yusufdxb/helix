"""GO2 preparation profile: real telemetry, isolated command topics, dry sink.

This entrypoint reuses the existing closed-loop stack without changing its
simulation or recovery behavior. Recovery stays disabled and unconfigured;
the arbiter therefore holds its isolated output at zero. The sink can only
trace decisions in dry_run and never creates a Sport API request publisher.

No launch argument enables recovery, changes command topics, or arms the
sink. Real command-path validation remains the separate, operator-gated
procedure in docs/HW_MOTION_TEST.md. Deployment scripts do not launch this
profile automatically.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("helix_bringup")
    arbiter_share = get_package_share_directory("helix_arbiter")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "helix_closedloop.launch.py")
        ),
        launch_arguments={
            "sim_mode": "false",
            "enable_twist_mux": "false",
            "sense_auto_activate": "true",
            "auto_activate_diagnosis": "true",
            "auto_activate_recovery": "false",
            "recovery_enabled": "false",
            "llm_enabled": "false",
            "arbiter_config": os.path.join(arbiter_share, "config", "arbiter_dry.yaml"),
            "cmd_vel_out": "/helix_dry/cmd_vel",
        }.items(),
    )
    sink = Node(
        package="helix_arbiter",
        executable="helix_go2_sport_sink",
        name="helix_go2_sport_sink",
        namespace="",
        parameters=[{
            "mode": "dry_run",
            "input_topic": "/helix_dry/cmd_vel",
        }],
        output="screen",
    )
    return LaunchDescription([stack, sink])
