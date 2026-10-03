"""HELIX closed-loop (SENSE + ADAPTER + DIAGNOSE + RECOVER + EXPLAIN) bringup.

Single entrypoint for the runtime reliability stack. Composes the existing
``helix_sensing.launch.py`` and ``helix_adapter.launch.py`` (both of which
already auto-configure and auto-activate their lifecycle nodes), and then
adds the three closed-loop-tier nodes: diagnosis (context_buffer + state
machine), recovery (enabled flag + allowlist envelope), and the LLM
explainer (advisory, llm_enabled gated).

Safety-relevant defaults:
    recovery_enabled:=false     : actuation path off unless operator asks
    llm_enabled:=false          : explainer runs template-only
    anomaly_backend:=           : Python anomaly detector (cpp selects the C++ port;
                                  legacy use_cpp_anomaly:=true still works)
    arbiter_backend:=python     : Python arbiter (cpp selects helix_arbiter_cpp)

Typical operator sequence on live hardware:
    # bring up the whole stack, recovery held off
    ros2 launch helix_bringup helix_closedloop.launch.py

    # operator verifies body_height > 0.25m at the robot, then:
    ros2 param set /helix_recovery_node enabled true
    ros2 param set /helix_recovery_node allowed_actions "['STOP_AND_HOLD', 'LOG_ONLY']"
    ros2 lifecycle set /helix_recovery_node configure
    ros2 lifecycle set /helix_recovery_node activate

This launcher auto-activates diagnosis by default, but recovery comes up in
``unconfigured`` so the operator can gate the actuation path on body-height
verification. Sense and adapter auto-activate as they always have.

Motion path (docs/MOTION_ARBITRATION.md): by default ``helix_arbiter`` is the
single authoritative publisher of ``cmd_vel_out``. HELIX recovery enters it
through the /helix/hold state, never as a velocity source. The arbiter is
auto-activated: with no live HELIX state it publishes zero, so it is safe.
The GO2 sport sink is NOT started here; the operator starts it per hardware
stage with an explicit mode (dry_run / stop_only / armed).

``arbiter_backend:=cpp`` starts the C++ arbiter (helix_arbiter_cpp) in place
of the Python one: same node name, parameters, topics and QoS. Exactly one
arbiter runs.

``enable_twist_mux:=true`` selects the legacy twist_mux path instead (used by
the Isaac Sim closure scenario). It is mutually exclusive with the arbiter,
so combining it with ``arbiter_backend:=cpp`` stops the launch. twist_mux goes
silent (does not publish zero) on idle and on lock, and passes NaN through;
it is not a safety layer.
"""
import os

import launch.events
from ament_index_python.packages import get_package_share_directory
from helix_bringup.backends import executable, resolve_arbiter_backend
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
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
    """Configure-then-activate sequence, mirroring helix_sensing.launch.py.

    The activate handler is registered before configure is emitted, so a fast
    configure transition cannot be missed.
    """
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
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=condition,
    )
    return [activate_on_inactive, configure]


def arbiter_actions(context, *args, **kwargs):
    """Start at most one arbiter: the selected backend, or none under twist_mux."""
    backend = resolve_arbiter_backend(
        LaunchConfiguration("arbiter_backend").perform(context),
        LaunchConfiguration("enable_twist_mux").perform(context),
    )
    if backend is None:
        return [LogInfo(msg="[helix_closedloop] enable_twist_mux:=true, no arbiter")]
    package, exe = executable("arbiter", backend)
    node = LifecycleNode(
        package=package,
        executable=exe,
        name="helix_arbiter",
        namespace="",
        parameters=[LaunchConfiguration("arbiter_config"),
                    {"output_topic": LaunchConfiguration("cmd_vel_out")}],
        output="screen",
    )
    # The arbiter is auto-activated: with no live HELIX state it publishes
    # zero, so it is safe.
    return [
        LogInfo(msg=f"[helix_closedloop] arbiter backend: {backend} ({package})"),
        node,
        *_auto_activate(node, None),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("helix_bringup")
    sensing_launch = os.path.join(bringup_share, "launch", "helix_sensing.launch.py")
    adapter_launch = os.path.join(bringup_share, "launch", "helix_adapter.launch.py")
    twist_mux_config = os.path.join(bringup_share, "config", "twist_mux.yaml")

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
            "Auto-configure/activate the recovery node. SAFETY-RELEVANT: "
            "recovery owns the /helix/hold state; while it is not active the "
            "arbiter holds the output at zero. Default false so the operator "
            "can gate on body_height verification at the robot first."
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
            "Default false, explainer emits deterministic templates only."
        ),
    )
    enable_twist_mux_arg = DeclareLaunchArgument(
        "enable_twist_mux",
        default_value="false",
        description=(
            "LEGACY. Use twist_mux (fed by /helix/cmd_vel zero-twist) instead "
            "of helix_arbiter. Mutually exclusive with the arbiter."
        ),
    )
    arbiter_config_arg = DeclareLaunchArgument(
        "arbiter_config",
        default_value=os.path.join(
            get_package_share_directory("helix_arbiter"), "config", "arbiter.yaml"),
        description="helix_arbiter parameter file.",
    )
    cmd_vel_out_arg = DeclareLaunchArgument(
        "cmd_vel_out",
        default_value="/cmd_vel",
        description=(
            "Authoritative velocity output (arbiter, or legacy twist_mux). "
            "Consumed by helix_go2_sport_sink on the robot (a stock GO2 has "
            "no /cmd_vel consumer) and by the Isaac Sim bridge."
        ),
    )
    arbiter_backend_arg = DeclareLaunchArgument(
        "arbiter_backend",
        default_value="python",
        description=(
            "Arbiter backend: 'python' (helix_arbiter) or 'cpp' "
            "(helix_arbiter_cpp). Refused together with enable_twist_mux:=true."
        ),
    )
    anomaly_backend_arg = DeclareLaunchArgument(
        "anomaly_backend",
        default_value="",
        description="Forwarded to helix_sensing.launch.py: python, cpp, or empty.",
    )
    use_cpp_anomaly_arg = DeclareLaunchArgument(
        "use_cpp_anomaly",
        default_value="false",
        description="Forwarded to helix_sensing.launch.py (legacy alias for cpp).",
    )
    twist_mux_config_arg = DeclareLaunchArgument(
        "twist_mux_config",
        default_value=twist_mux_config,
        description=(
            "Path to the twist_mux YAML. Defaults to the package-installed "
            "config/twist_mux.yaml."
        ),
    )

    # --- sense + adapter (re-use existing launchers) --------------------------
    sense_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensing_launch),
        launch_arguments={
            "auto_activate": LaunchConfiguration("sense_auto_activate"),
            "anomaly_backend": LaunchConfiguration("anomaly_backend"),
            "use_cpp_anomaly": LaunchConfiguration("use_cpp_anomaly"),
        }.items(),
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

    # --- recovery tier (actuation, gated) -------------------------------------
    recovery_node = LifecycleNode(
        package="helix_recovery",
        executable="helix_recovery_node",
        name="helix_recovery_node",
        namespace="",
        parameters=[{
            "enabled": LaunchConfiguration("recovery_enabled"),
            "cooldown_seconds": LaunchConfiguration("recovery_cooldown_seconds"),
            "publish_legacy_cmd_vel": LaunchConfiguration("enable_twist_mux"),
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

    # --- twist_mux arbiter ----------------------------------------------------
    # Subscribes to /teleop/cmd_vel (priority 200), /helix/cmd_vel (100),
    # /nav/cmd_vel (50). Publishes the winning twist on cmd_vel_out
    # (default /cmd_vel). twist_mux's upstream output topic is named
    # cmd_vel_out, which we remap to the configurable cmd_vel_out arg so
    # the GO2 sport-mode bridge and the Isaac Sim bridge both pick it up.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        namespace="",
        parameters=[LaunchConfiguration("twist_mux_config")],
        remappings=[("cmd_vel_out", LaunchConfiguration("cmd_vel_out"))],
        condition=IfCondition(LaunchConfiguration("enable_twist_mux")),
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
        enable_twist_mux_arg,
        cmd_vel_out_arg,
        twist_mux_config_arg,
        arbiter_config_arg,
        arbiter_backend_arg,
        anomaly_backend_arg,
        use_cpp_anomaly_arg,
        sense_include,
        adapter_include,
        context_buffer,
        diagnosis_node,
        recovery_node,
        llm_explainer,
        twist_mux_node,
        # helix_arbiter: the single authoritative motion output.
        OpaqueFunction(function=arbiter_actions),
    ]
    actions.extend(_auto_activate(context_buffer, diag_cond))
    actions.extend(_auto_activate(diagnosis_node, diag_cond))
    actions.extend(_auto_activate(recovery_node, recov_cond))
    return LaunchDescription(actions)
