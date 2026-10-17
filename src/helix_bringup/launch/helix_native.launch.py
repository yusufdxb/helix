"""HELIX native runtime: the C++ monitoring, supervision and arbitration path.

One process per node, all lifecycle nodes auto-activated:

  helix_topic_rate_monitor  (helix_sensing_cpp)  rate_hz/* on /helix/metrics
  helix_anomaly_detector    (helix_sensing_cpp)  FaultEvent + clear records
  helix_log_parser          (helix_core, Python) LOG_PATTERN faults from /rosout
  helix_supervisor          (helix_supervisor_cpp) heartbeat registry, fault
                            state machine, hold, controller-ack tracking
  helix_arbiter             (helix_arbiter_cpp)  sole velocity publisher
  helix_go2_sport_sink      (helix_arbiter, Python) arbiter -> Sport API,
                            StopMove ack + measured motion on /helix/sink/status

The supervisor replaces helix_heartbeat_monitor, helix_diagnosis_node,
helix_context_buffer and helix_recovery_node. Python stays for offline
analysis, tests and the optional LLM explainer, none of which run here.

Safe by default: sink_mode:=dry_run sends nothing to the robot, and the
arbiter and its sources live on /helix_dry/* (arbiter_dry.yaml). The
supervisor starts with actuation_enabled:=false, so a fault is detected,
traced and acked but never held. Arming is three explicit choices:
    sink_mode:=armed arbiter_config:=<arbiter.yaml> actuation_enabled:=true
and moving still needs an upstream command source on the arbiter.
"""
import os

import launch.events
from ament_index_python.packages import get_package_share_directory
from helix_bringup.backends import parse_bool
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from launch import LaunchDescription

SINK_MODES = ('dry_run', 'stop_only', 'armed')

# Detector config chosen on DEV recordings and scored once on held-out ones
# (analysis/detector_eval.py, docs/DETECTOR_EVAL.md).
DETECTOR_OVERRIDES = {
    'zscore_threshold': 4.0,
    'window_size': 60,
    'consecutive_trigger': 3,
    'min_anomaly_duration_s': 1.0,
    'duration_tolerance_s': 0.25,
    'emit_cooldown_s': 1.0,
    'freeze_baseline': True,
    'std_floor_rel': 0.02,
    'clear_samples': 4,
    'anchor_tolerance_rel': 0.1,
    'anchor_metric_prefix': 'rate_hz/',
}

# Nodes that heartbeat on /helix/heartbeat in this launch. The supervisor
# holds in STARTING until they beat, and a node that never beats becomes a
# CRASH fault after startup_grace_s.
EXPECTED_NODES = ['helix_topic_rate_monitor', 'helix_anomaly_detector', 'helix_log_parser']


def _auto_activate(node: LifecycleNode):
    """Activate when the node reaches 'inactive', then emit configure."""
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        ),
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=launch.events.matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    return [activate_on_inactive, configure]


def _lifecycle(package, executable, name, parameters):
    node = LifecycleNode(package=package, executable=executable, name=name,
                         namespace='', parameters=parameters, output='screen')
    return [node, *_auto_activate(node)]


def native_actions(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    sink_mode = arg('sink_mode').strip()
    if sink_mode not in SINK_MODES:
        raise ValueError(f'sink_mode must be one of {list(SINK_MODES)}, got {sink_mode!r}')
    actuation = parse_bool('actuation_enabled', arg('actuation_enabled'))
    auto_resume = parse_bool('auto_resume', arg('auto_resume'))
    stale_after = float(arg('rate_stale_after_s'))
    bringup = get_package_share_directory('helix_bringup')
    params = os.path.join(bringup, 'config', 'helix_params.yaml')
    adapter_params = os.path.join(bringup, 'config', 'helix_adapter_params.yaml')
    rules = os.path.join(bringup, 'config', 'log_rules.yaml')
    arbiter_config = arg('arbiter_config')
    cmd_out = arg('cmd_vel_out')

    actions = [LogInfo(msg=(
        f'[helix_native] sink_mode={sink_mode} actuation_enabled={actuation} '
        f'auto_resume={auto_resume} arbiter_config={arbiter_config} cmd_vel_out={cmd_out}'))]
    actions += _lifecycle('helix_sensing_cpp', 'helix_topic_rate_monitor',
                          'helix_topic_rate_monitor',
                          [adapter_params, {'stale_after_s': stale_after}])
    actions += _lifecycle('helix_sensing_cpp', 'helix_anomaly_detector',
                          'helix_anomaly_detector', [params, DETECTOR_OVERRIDES])
    actions += _lifecycle('helix_core', 'helix_log_parser', 'helix_log_parser',
                          [params, {'rules_file_path': rules}])
    actions += _lifecycle('helix_supervisor_cpp', 'helix_supervisor', 'helix_supervisor', [{
        'expected_nodes': EXPECTED_NODES,
        'critical_nodes': ['*'],
        'actuation_enabled': actuation,
        'auto_resume': auto_resume,
    }])
    actions += _lifecycle('helix_arbiter_cpp', 'helix_arbiter', 'helix_arbiter',
                          [arbiter_config, {'output_topic': cmd_out}])
    actions.append(Node(
        package='helix_arbiter', executable='helix_go2_sport_sink',
        name='helix_go2_sport_sink', namespace='',
        parameters=[{'mode': sink_mode, 'input_topic': cmd_out,
                     'feedback': parse_bool('sink_feedback', arg('sink_feedback'))}],
        output='screen'))
    return actions


def generate_launch_description() -> LaunchDescription:
    arbiter_share = get_package_share_directory('helix_arbiter')
    return LaunchDescription([
        DeclareLaunchArgument(
            'sink_mode', default_value='dry_run',
            description='dry_run (nothing sent), stop_only (StopMove only) or armed.'),
        DeclareLaunchArgument(
            'actuation_enabled', default_value='false',
            description='Supervisor asserts /helix/hold on stop-worthy faults.'),
        DeclareLaunchArgument(
            'auto_resume', default_value='false',
            description='Release the hold automatically once every fault has cleared '
                        'for clear_hold_s (false: operator release service).'),
        DeclareLaunchArgument(
            'arbiter_config',
            default_value=os.path.join(arbiter_share, 'config', 'arbiter_dry.yaml'),
            description='helix_arbiter parameter file (default: /helix_dry/* topics).'),
        DeclareLaunchArgument(
            'cmd_vel_out', default_value='/helix_dry/cmd_vel',
            description='Arbiter output and sink input.'),
        DeclareLaunchArgument(
            'rate_stale_after_s', default_value='1.0',
            description='Rate monitor reports NaN once a topic is silent this long.'),
        DeclareLaunchArgument(
            'sink_feedback', default_value='true',
            description='Sink tracks /api/sport/response and SportModeState.'),
        OpaqueFunction(function=native_actions),
    ])
