"""HELIX Phase 1 sensing-stack bringup.

Starts the three lifecycle sensor nodes and (by default) auto-transitions them
through configure -> activate so the documented quick start produces an
actually-running stack, not three nodes parked in the ``unconfigured`` state.

The anomaly detector has two interchangeable backends with the same node
name, parameters and topics. Python (``helix_core``) is the default; the C++
port (``helix_sensing_cpp``) is selected with ``anomaly_backend:=cpp``. The
older ``use_cpp_anomaly:=true`` flag still selects C++; combining it with
``anomaly_backend:=python`` stops the launch instead of guessing:
    ros2 launch helix_bringup helix_sensing.launch.py anomaly_backend:=cpp

Pass ``auto_activate:=false`` if you want to drive the lifecycle by hand:
    ros2 launch helix_bringup helix_sensing.launch.py auto_activate:=false
    ros2 lifecycle set /helix_heartbeat_monitor configure
    ros2 lifecycle set /helix_heartbeat_monitor activate
    # repeat for helix_anomaly_detector and helix_log_parser
"""
import os

import launch.events
from ament_index_python.packages import get_package_share_directory
from helix_bringup.backends import executable, resolve_anomaly_backend
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
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
    """Activate when the node reaches 'inactive', then emit configure.

    The handler is registered before configure is emitted, so a configure
    transition that completes quickly cannot be missed.
    """
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
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=condition,
    )
    return [activate_on_inactive, configure]


def _params_file():
    return os.path.join(
        get_package_share_directory("helix_bringup"), "config", "helix_params.yaml")


def anomaly_detector_actions(context, *args, **kwargs):
    """Start exactly one anomaly detector, on the selected backend."""
    backend = resolve_anomaly_backend(
        LaunchConfiguration("anomaly_backend").perform(context),
        LaunchConfiguration("use_cpp_anomaly").perform(context),
    )
    package, exe = executable("anomaly_detector", backend)
    node = LifecycleNode(
        package=package,
        executable=exe,
        name="helix_anomaly_detector",
        namespace="",
        parameters=[_params_file()],
        output="screen",
    )
    return [
        LogInfo(msg=f"[helix_bringup] anomaly detector backend: {backend} ({package})"),
        node,
        *_auto_activate(node, IfCondition(LaunchConfiguration("auto_activate"))),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("helix_bringup")
    params_file = _params_file()
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
    anomaly_backend_arg = DeclareLaunchArgument(
        "anomaly_backend",
        default_value="",
        description=(
            "Anomaly detector backend: 'python' (helix_core) or 'cpp' "
            "(helix_sensing_cpp). Empty (default) defers to use_cpp_anomaly, "
            "which defaults to python."
        ),
    )
    use_cpp_anomaly_arg = DeclareLaunchArgument(
        "use_cpp_anomaly",
        default_value="false",
        description="Legacy alias: true selects anomaly_backend cpp.",
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

    actions = [
        auto_activate_arg,
        anomaly_backend_arg,
        use_cpp_anomaly_arg,
        heartbeat_monitor,
        OpaqueFunction(function=anomaly_detector_actions),
        log_parser,
    ]
    for node in (heartbeat_monitor, log_parser):
        actions.extend(_auto_activate(node, auto_activate_cond))
    return LaunchDescription(actions)
