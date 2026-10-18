"""HELIX native runtime: the C++ monitoring, supervision and arbitration path.

One process per node, all lifecycle nodes auto-activated:

  helix_topic_rate_monitor  (helix_sensing_cpp)  rate_hz/* on /helix/metrics
  helix_anomaly_detector    (helix_sensing_cpp)  FaultEvent + clear records
  helix_log_parser          (helix_sensing_cpp)  LOG_PATTERN faults from /rosout
  helix_supervisor          (helix_supervisor_cpp) heartbeat registry, fault
                            state machine, hold, controller-ack tracking
  helix_arbiter             (helix_arbiter_cpp)  sole velocity publisher
  helix_go2_sport_sink      (helix_go2_sink_cpp; sink_backend:=python for the
                            reference) arbiter -> Sport API, StopMove ack and
                            measured motion on /helix/sink/status

The supervisor replaces helix_heartbeat_monitor, helix_diagnosis_node,
helix_context_buffer and helix_recovery_node. No Python runs on the robot
in this profile except the launch process itself (and the reference sink
with sink_backend:=python). Python stays for offline
analysis, tests and the optional LLM explainer, none of which run here.

Safe by default: sink_mode:=dry_run sends nothing to the robot, and the
arbiter and its sources live on /helix_dry/* (arbiter_dry.yaml). The
supervisor starts with actuation_enabled:=false. The arbiter independently
starts protectively latched and requires health plus operator release.
Arming is three explicit choices:
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

# Detector config chosen on DEV recordings (analysis/detector_eval.py round 2,
# docs/DETECTOR_EVAL.md). It passed a short held-out test (0.216 h).
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

# Fixed off-robot profile. Canonical sensor parameter names remain unchanged so
# rate_hz metric labels keep the native detector contract after ROS remapping.
GUARD_SIM_BASE = '/helix/experimental/native'
# Eight child processes plus lifecycle launch and test driver exceed Cyclone's
# default auto-participant range. ROS_LOCALHOST_ONLY supplies the loopback
# interface; overriding its Interfaces section conflicts with the RMW config.
GUARD_SIM_CYCLONE_URI = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery>'
    '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>'
    '<Peers><Peer Address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
)
GUARD_SIM_TOPICS = {
    name: GUARD_SIM_BASE + '/' + name for name in (
        'heartbeat', 'faults', 'fault_clear', 'hold', 'metrics', 'diagnostics', 'rosout',
        'arbiter/status_raw', 'arbiter/status', 'supervisor/status', 'supervisor/release',
        'recovery_hints', 'recovery_actions', 'cmd_vel', 'nav/cmd_vel', 'teleop/cmd_vel',
        'sport/intent_raw', 'sport/intent', 'sport/feedback', 'sport/sink_feedback',
        'sport/sink_trace', 'sport/guard_status', 'sport/release',
        'controller/request_raw', 'controller/request',
        'controller/response_raw', 'controller/response',
        'utlidar/imu', 'utlidar/robot_odom', 'utlidar/robot_pose', 'utlidar/cloud',
        'gnss', 'multiplestate', 'sportmodestate', 'lf/sportmodestate',
    )
}
GUARD_SIM_REMAPS = [
    ('/helix/' + original, GUARD_SIM_TOPICS[target])
    for original, target in (
        ('heartbeat', 'heartbeat'), ('faults', 'faults'), ('fault_clear', 'fault_clear'),
        ('hold', 'hold'), ('metrics', 'metrics'), ('arbiter/status', 'arbiter/status_raw'),
        ('supervisor/status', 'supervisor/status'), ('supervisor/release', 'supervisor/release'),
        ('recovery_hints', 'recovery_hints'), ('recovery_actions', 'recovery_actions'),
    )
] + [
    ('/' + topic, GUARD_SIM_TOPICS[topic]) for topic in (
        'rosout', 'diagnostics', 'utlidar/imu', 'utlidar/robot_odom', 'utlidar/robot_pose',
        'utlidar/cloud', 'gnss', 'multiplestate', 'sportmodestate', 'lf/sportmodestate',
        'nav/cmd_vel', 'teleop/cmd_vel',
    )
]


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


def _lifecycle(package, executable, name, parameters, remappings=None):
    options = {} if remappings is None else {'remappings': remappings}
    node = LifecycleNode(package=package, executable=executable, name=name,
                         namespace='', parameters=parameters, output='screen', **options)
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
        f'[helix_native] sink_mode={sink_mode} sink_backend={arg("sink_backend")} '
        f'actuation_enabled={actuation} '
        f'auto_resume={auto_resume} arbiter_config={arbiter_config} cmd_vel_out={cmd_out}'))]
    actions += _lifecycle('helix_sensing_cpp', 'helix_topic_rate_monitor',
                          'helix_topic_rate_monitor',
                          [adapter_params, {'stale_after_s': stale_after}])
    actions += _lifecycle('helix_sensing_cpp', 'helix_anomaly_detector',
                          'helix_anomaly_detector', [params, DETECTOR_OVERRIDES])
    actions += _lifecycle('helix_sensing_cpp', 'helix_log_parser', 'helix_log_parser',
                          [params, {'rules_file_path': rules}])
    actions += _lifecycle('helix_supervisor_cpp', 'helix_supervisor', 'helix_supervisor', [{
        'expected_nodes': EXPECTED_NODES,
        'critical_nodes': ['*'],
        'actuation_enabled': actuation,
        'auto_resume': auto_resume,
    }])
    actions += _lifecycle('helix_arbiter_cpp', 'helix_arbiter', 'helix_arbiter',
                          [arbiter_config, {'output_topic': cmd_out, 'protective_latch': True}])
    sink_backend = arg('sink_backend').strip()
    if sink_backend not in ('cpp', 'python'):
        raise ValueError(f"sink_backend must be 'cpp' or 'python', got {sink_backend!r}")
    sink_pkg = 'helix_go2_sink_cpp' if sink_backend == 'cpp' else 'helix_arbiter'
    actions.append(Node(
        package=sink_pkg, executable='helix_go2_sport_sink',
        name='helix_go2_sport_sink', namespace='',
        parameters=[{'mode': sink_mode, 'input_topic': cmd_out,
                     'feedback': parse_bool('sink_feedback', arg('sink_feedback'))}],
        output='screen'))
    return actions


def guard_sim_node_specs():
    """Return the fixed native/guard/FakeGo2 parameters for launch and restarts.

    The caller supplies four communication relays to exercise loss between
    intent_raw/intent, arbiter status_raw/status, controller request_raw/request,
    and controller response_raw/response. No relay is started by this profile.
    """
    bringup = get_package_share_directory('helix_bringup')
    arbiter = get_package_share_directory('helix_arbiter')
    params = os.path.join(bringup, 'config', 'helix_params.yaml')
    adapter_params = os.path.join(bringup, 'config', 'helix_adapter_params.yaml')
    rules = os.path.join(bringup, 'config', 'log_rules.yaml')
    topics = GUARD_SIM_TOPICS
    specs = []

    def add(package, executable, parameters, lifecycle=True, name=None, remappings=None):
        specs.append({
            'package': package, 'executable': executable, 'name': name or executable,
            'parameters': parameters, 'lifecycle': lifecycle,
            'remappings': list(GUARD_SIM_REMAPS if remappings is None else remappings),
        })

    add('helix_sensing_cpp', 'helix_topic_rate_monitor',
        [adapter_params, {'stale_after_s': 1.0}])
    add('helix_sensing_cpp', 'helix_anomaly_detector', [params, DETECTOR_OVERRIDES])
    add('helix_sensing_cpp', 'helix_log_parser', [params, {'rules_file_path': rules}])
    add('helix_supervisor_cpp', 'helix_supervisor', [{
        'expected_nodes': EXPECTED_NODES + ['helix_sport_guard'],
        'critical_nodes': ['*'], 'actuation_enabled': True, 'auto_resume': False,
        'require_controller_ack': True,
        'hold_topic': topics['hold'],
        'arbiter_status_topic': topics['arbiter/status_raw'],
        'sink_status_topic': topics['sport/feedback'],
    }])
    add('helix_arbiter_cpp', 'helix_arbiter', [
        os.path.join(arbiter, 'config', 'arbiter.yaml'), {
            'output_topic': topics['cmd_vel'], 'protective_latch': True,
            'hold_topic': topics['hold'], 'status_topic': topics['arbiter/status_raw'],
        },
    ])
    add('helix_go2_sink_cpp', 'helix_go2_sport_sink', [{
        'mode': 'armed', 'input_topic': topics['cmd_vel'], 'feedback': True,
        'request_topic': topics['sport/intent_raw'],
        'response_topic': topics['controller/response'],
        'state_topic': topics['lf/sportmodestate'],
        'status_topic': topics['sport/sink_feedback'],
        'trace_topic': topics['sport/sink_trace'],
    }], lifecycle=False)
    add('helix_go2_sink_cpp', 'helix_sport_guard', [{
        'mode': 'armed', 'intent_topic': topics['sport/intent'],
        'request_topic': topics['controller/request_raw'],
        'response_topic': topics['controller/response'],
        'arbiter_status_topic': topics['arbiter/status'],
        'fault_topic': topics['faults'], 'heartbeat_topic': topics['heartbeat'],
        'status_topic': topics['sport/guard_status'], 'release_topic': topics['sport/release'],
        'feedback_enabled': True, 'feedback_topic': topics['sport/feedback'],
        'state_topic': topics['lf/sportmodestate'],
        'controller_ack_watchdog': True, 'controller_ack_timeout_s': 0.3,
        'controller_state_timeout_s': 0.3,
    }], lifecycle=False)
    add('helix_arbiter', 'helix_fake_go2', [{'move_timeout_s': 0.0}],
        lifecycle=False, name='fake_go2', remappings=GUARD_SIM_REMAPS + [
            ('/api/sport/request', topics['controller/request']),
            ('/api/sport/response', topics['controller/response_raw']),
        ])
    return specs


def guard_sim_actions():
    """Build the fixed isolated simulation graph, without communication relays."""
    actions = [LogInfo(msg='[helix_native_guard_sim] LOCALHOST SIMULATION ONLY; '
                       'native supervision, protective arbitration, guard and FakeGo2')]
    for spec in guard_sim_node_specs():
        if spec['lifecycle']:
            actions += _lifecycle(
                spec['package'], spec['executable'], spec['name'],
                spec['parameters'], spec['remappings'])
        else:
            actions.append(Node(
                package=spec['package'], executable=spec['executable'], name=spec['name'],
                namespace='', parameters=spec['parameters'],
                remappings=spec['remappings'], output='screen'))
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
            description='Clear the supervisor episode automatically after clear_hold_s. '
                        'The arbiter protective latch still requires operator release.'),
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
            'sink_backend', default_value='cpp',
            description="'cpp' (helix_go2_sink_cpp) or 'python' (helix_arbiter)."),
        DeclareLaunchArgument(
            'sink_feedback', default_value='true',
            description='Sink tracks /api/sport/response and SportModeState.'),
        OpaqueFunction(function=native_actions),
    ])
