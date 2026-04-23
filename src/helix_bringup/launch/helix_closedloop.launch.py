"""HELIX closed-loop (SENSE + ADAPTER + DIAGNOSE + RECOVER + EXPLAIN) bringup.

Single entrypoint for the self-healing stack. Composes the existing
``helix_sensing.launch.py`` and ``helix_adapter.launch.py`` (both of which
already auto-configure and auto-activate their lifecycle nodes), and then
adds the three closed-loop-tier nodes: diagnosis (context_buffer + state
machine), recovery (enabled flag + allowlist envelope), and the LLM
explainer (advisory, llm_enabled gated).

Safety-relevant defaults:
    recovery_enabled:=false     — actuation path off unless operator asks
    llm_enabled:=false          — explainer runs template-only
    use_cpp_anomaly:=false      — Python anomaly detector path

Typical operator sequence on live hardware:
    # bring up the whole stack, recovery held off
    ros2 launch helix_bringup helix_closedloop.launch.py

    # operator verifies body_height > 0.25m at the robot, then:
    ros2 param set /helix_recovery_node enabled true
    ros2 lifecycle set /helix_recovery_node configure
    ros2 lifecycle set /helix_recovery_node activate

This launcher does NOT auto-activate the diagnosis or recovery nodes — they
come up in ``unconfigured`` so the operator can gate the actuation path on
body_height verification. Sense + adapter auto-activate as they always have.
"""
import os

import launch.events
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from launch import LaunchDescription


def _auto_activate(node: LifecycleNode, condition):
    """Configure-then-activate sequence, mirroring helix_sensing.launch.py."""
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
                LogInfo(msg="[helix_closedloop] auto-activating lifecycle node"),
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
    sensing_launch = os.path.join(bringup_share, "launch", "helix_sensing.launch.py")
    adapter_launch = os.path.join(bringup_share, "launch", "helix_adapter.launch.py")

    # --- launch args ----------------------------------------------------------
    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description="Remap /utlidar/cloud to /utlidar/cloud_throttled for Isaac Sim.",
    )
    sense_auto_activate_arg = DeclareLaunchArgument(
        "sense_auto_activate",
        default_value="true",
        description="Auto-configure/activate SENSE + ADAPTER lifecycle nodes.",
    )
    auto_activate_diagnosis_arg = DeclareLaunchArgument(
        "auto_activate_diagnosis",
        default_value="true",
        description=(
            "Auto-configure/activate diagnosis (context_buffer + state machine). "
            "Diagnosis is non-actuating; safe to auto-activate."
        ),
    )
    auto_activate_recovery_arg = DeclareLaunchArgument(
        "auto_activate_recovery",
        default_value="false",
        description=(
            "Auto-configure/activate the recovery node. SAFETY-RELEVANT — "
            "recovery is the only node that publishes cmd_vel. Default false "
            "so the operator can gate on body_height verification at the robot "
            "before enabling the actuation path."
        ),
    )
    recovery_enabled_arg = DeclareLaunchArgument(
        "recovery_enabled",
        default_value="false",
        description=(
            "Value of the /helix_recovery_node.enabled parameter at launch. "
            "false = envelope suppresses every hint. Operator sets true via "
            "ros2 param set or relaunch once robot is verified safe."
        ),
    )
    recovery_cooldown_arg = DeclareLaunchArgument(
        "recovery_cooldown_seconds",
        default_value="5.0",
        description="Per-fault cooldown before a repeat action is accepted.",
    )
    llm_enabled_arg = DeclareLaunchArgument(
        "llm_enabled",
        default_value="false",
        description=(
            "If true, the explainer calls the local llama-server sidecar. "
            "Default false — explainer emits deterministic templates only."
        ),
    )

    # --- sense + adapter (re-use existing launchers) --------------------------
    sense_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensing_launch),
        launch_arguments={"auto_activate": LaunchConfiguration("sense_auto_activate")}.items(),
    )
    adapter_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(adapter_launch),
        launch_arguments={
            "auto_activate": LaunchConfiguration("sense_auto_activate"),
            "sim_mode": LaunchConfiguration("sim_mode"),
        }.items(),
    )

    # --- diagnosis tier -------------------------------------------------------
    context_buffer = LifecycleNode(
        package="helix_diagnosis",
        executable="helix_context_buffer",
        name="helix_context_buffer",
        namespace="",
        output="screen",
    )
    diagnosis_node = LifecycleNode(
        package="helix_diagnosis",
        executable="helix_diagnosis_node",
        name="helix_diagnosis_node",
        namespace="",
        output="screen",
    )

    # --- recovery tier (actuation — gated) ------------------------------------
    recovery_node = LifecycleNode(
        package="helix_recovery",
        executable="helix_recovery_node",
        name="helix_recovery_node",
        namespace="",
        parameters=[{
            "enabled": LaunchConfiguration("recovery_enabled"),
            "cooldown_seconds": LaunchConfiguration("recovery_cooldown_seconds"),
        }],
        output="screen",
    )

    # --- explanation tier (advisory, non-lifecycle) ---------------------------
    llm_explainer = Node(
        package="helix_explanation",
        executable="helix_llm_explainer",
        name="helix_llm_explainer",
        namespace="",
        parameters=[{"llm_enabled": LaunchConfiguration("llm_enabled")}],
        output="screen",
    )

    # --- lifecycle auto-activation -------------------------------------------
    diag_cond = IfCondition(LaunchConfiguration("auto_activate_diagnosis"))
    recov_cond = IfCondition(LaunchConfiguration("auto_activate_recovery"))

    actions = [
        sim_mode_arg,
        sense_auto_activate_arg,
        auto_activate_diagnosis_arg,
        auto_activate_recovery_arg,
        recovery_enabled_arg,
        recovery_cooldown_arg,
        llm_enabled_arg,
        sense_include,
        adapter_include,
        context_buffer,
        diagnosis_node,
        recovery_node,
        llm_explainer,
    ]
    actions.extend(_auto_activate(context_buffer, diag_cond))
    actions.extend(_auto_activate(diagnosis_node, diag_cond))
    actions.extend(_auto_activate(recovery_node, recov_cond))
    return LaunchDescription(actions)
