"""Experimental localhost last-hop guard against a persistent-Move controller.

The fake controller deliberately retains its last Move indefinitely, including
after every HELIX process dies. Its StopMove can also be denied or dropped.
Two additional cases run the project's FakeGo2 with command expiry disabled.
Arbiter statuses here model the trusted protective challenge/release protocol;
the existing native crash suite separately tests supervisor/operator authority.
These are simulation observations, never physical stopping evidence.
"""
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('unitree_api')
pytest.importorskip('helix_msgs')


class Rig:
    """Own isolated ROS processes and a controller with no command expiry."""

    def __init__(self, tmp_path):
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from std_msgs.msg import String
        from unitree_api.msg import Request, Response

        from helix_msgs.msg import ArbiterStatus, FaultEvent, SinkStatus

        self.rclpy, self.Request, self.Response = rclpy, Request, Response
        self.String = String
        self.ArbiterStatus, self.Twist = ArbiterStatus, Twist
        self.env = dict(os.environ, ROS_LOCALHOST_ONLY='1',
                        RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                        ROS_DOMAIN_ID=str(150 + os.getpid() % 50))
        self.env.pop('CYCLONEDDS_URI', None)
        self.old_cyclone_uri = os.environ.pop('CYCLONEDDS_URI', None)
        self.old_localhost = os.environ.get('ROS_LOCALHOST_ONLY')
        self.old_rmw = os.environ.get('RMW_IMPLEMENTATION')
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        rclpy.init(domain_id=int(self.env['ROS_DOMAIN_ID']))
        self.node = Node('persistent_sport_controller_test')
        self.tmp_path = tmp_path
        self.processes, self.logs = [], []
        prefix = (Path(get_package_prefix('helix_go2_sink_cpp'))
                  / 'lib/helix_go2_sink_cpp')
        self.guard_binary = prefix / 'helix_sport_guard'
        self.sink_binary = prefix / 'helix_go2_sport_sink'
        if not self.guard_binary.exists() or not self.sink_binary.exists():
            self.close()
            pytest.skip('build sink package with unitree_api/unitree_go first')
        self.base = '/helix/experimental/'
        self.requests, self.responses = [], []
        self.intents, self.faults = [], []
        self.native_faults = []
        self.guard_status, self.sink_status = None, None
        self.target = 0.0
        self.controller_enabled = True
        self.odom = []
        self.stop_policy = 'accept'
        self.command = 0.0
        self.streaming = True
        self.arbiter_protected = True
        self.arbiter_generation = 1
        self.arbiter_seq = 0
        self.auto_protect = True
        self.status_streaming = True
        self.response_pub = self.node.create_publisher(
            Response, self.base + 'sport/response', 50)
        self.node.create_subscription(
            Request, self.base + 'sport/request', self.on_request, 50)
        self.node.create_subscription(
            Request, self.base + 'sport/intent', self.on_intent, 50)
        self.node.create_subscription(
            String, self.base + 'sport/guard_status', self.on_guard_status, 50)
        self.node.create_subscription(
            SinkStatus, self.base + 'sink/status', self.on_sink_status, 50)
        self.node.create_subscription(
            FaultEvent, self.base + 'faults', self.on_fault, 50)
        self.node.create_subscription(
            FaultEvent, '/helix/faults', self.native_faults.append, 50)
        self.node.create_subscription(
            Odometry, self.base + 'robot_odom', self.on_odom, 50)
        self.command_pub = self.node.create_publisher(
            Twist, self.base + 'sink/cmd_vel', 20)
        self.intent_pub = self.node.create_publisher(
            Request, self.base + 'sport/intent', 20)
        self.arbiter_pub = self.node.create_publisher(
            ArbiterStatus, self.base + 'arbiter/status', 20)
        self.release_pub = self.node.create_publisher(
            String, self.base + 'sport/release', 10)
        self.node.create_timer(0.02, self.tick)

    def tick(self):
        if self.streaming:
            command = self.Twist()
            command.linear.x = self.command
            self.command_pub.publish(command)
        if self.status_streaming:
            self.arbiter_seq += 1
            status = self.ArbiterStatus()
            status.seq = self.arbiter_seq
            status.release_challenge = (
                f'test-arbiter:{self.arbiter_generation}')
            status.protective_latched = self.arbiter_protected
            status.hold_active = self.arbiter_protected
            self.arbiter_pub.publish(status)

    def on_request(self, request):
        api = request.header.identity.api_id
        request_id = request.header.identity.id
        self.requests.append(
            (time.monotonic(), api, request_id, request.parameter))
        assert api in (1003, 1008), 'Damp and posture APIs are forbidden'
        if not self.controller_enabled:
            return
        code = 0
        if api == 1008:
            self.target = json.loads(request.parameter)['x']
        elif self.stop_policy == 'accept':
            self.target = 0.0
        elif self.stop_policy == 'drop':
            return
        else:
            code = -1
        response = self.Response()
        response.header.identity.id = request_id
        response.header.identity.api_id = api
        response.header.status.code = code
        self.responses.append((api, request_id, code))
        self.response_pub.publish(response)

    def on_intent(self, request):
        self.intents.append(
            (request.header.identity.api_id, request.header.identity.id))

    def on_guard_status(self, status):
        self.guard_status = json.loads(status.data)

    def on_sink_status(self, status):
        self.sink_status = status

    def on_odom(self, odometry):
        self.odom.append((time.monotonic(), odometry.twist.twist.linear.x))

    def on_fault(self, fault):
        self.faults.append(fault)
        if self.auto_protect and not self.arbiter_protected:
            self.protect()

    def protect(self):
        self.arbiter_generation += 1
        self.arbiter_protected = True

    def release(self, guard_authorize=True):
        """Explicitly release the trusted arbiter, then this guard instance."""
        self.arbiter_protected = False
        if guard_authorize:
            self.wait(lambda: self.guard_status
                      and self.guard_status['release_ready'])
            self.release_guard()

    def release_guard(self, token=None):
        if token is None:
            token = self.guard_status['release_token']
        self.release_pub.publish(self.String(data=token))
        self.spin(0.06)

    def start(self, kind, protected=True, extra=None, env=None):
        if kind == 'guard':
            binary = self.guard_binary
            params = {'mode': 'armed'}
        else:
            binary = self.sink_binary
            params = {
                'mode': 'armed', 'input_topic': self.base + 'sink/cmd_vel',
                'request_topic': (self.base + 'sport/'
                                  + ('intent' if protected else 'request')),
                'response_topic': self.base + 'sport/response',
                'state_topic': self.base + 'unpublished/sportmodestate',
                'status_topic': self.base + 'sink/status',
                'trace_topic': self.base + 'sink/trace',
            }
        params.update(extra or {})
        argv = [str(binary), '--ros-args']
        for key, value in params.items():
            argv += ['-p', f'{key}:={value}']
        return self.spawn(kind, argv, env)

    def spawn(self, kind, argv, env=None):
        log = (self.tmp_path / f'{kind}-{len(self.processes)}.log').open('w')
        self.logs.append(log)
        process = subprocess.Popen(argv, env=env or self.env,
                                   stdout=log, stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def start_fake_go2(self):
        """Run the real FakeGo2, disabling the toy responder entirely."""
        from ament_index_python.packages import get_package_prefix

        binary = (Path(get_package_prefix('helix_arbiter'))
                  / 'lib/helix_arbiter/helix_fake_go2')
        if not binary.exists():
            pytest.skip('install helix_arbiter FakeGo2 first')
        self.controller_enabled = False
        self.node.destroy_publisher(self.response_pub)
        self.response_pub = None
        argv = [str(binary), '--ros-args', '-p', 'move_timeout_s:=0.0']
        for original, private in [
                ('/api/sport/request', 'sport/request'),
                ('/api/sport/response', 'sport/response'),
                ('/utlidar/robot_odom', 'robot_odom'),
                ('/lf/sportmodestate', 'sportmodestate')]:
            argv += ['-r', f'{original}:={self.base}{private}']
        return self.spawn('fake_go2', argv)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            if predicate():
                return
        logs = '\n'.join(
            Path(log.name).read_text()[-3000:] for log in self.logs)
        pytest.fail(
            f'condition timed out; guard={self.guard_status}; logs={logs}')

    def enable(self):
        self.wait(lambda: self.guard_status and self.guard_status['latched']
                  and not self.guard_status['fault_required'] and self.intents)
        self.release()
        self.wait(lambda: self.target == 0.2)

    def kill(self, process):
        process.kill()
        process.wait(timeout=3)
        return time.monotonic()

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        for log in self.logs:
            log.close()
        self.node.destroy_node()
        self.rclpy.shutdown()
        if self.old_localhost is None:
            os.environ.pop('ROS_LOCALHOST_ONLY', None)
        else:
            os.environ['ROS_LOCALHOST_ONLY'] = self.old_localhost
        if self.old_rmw is None:
            os.environ.pop('RMW_IMPLEMENTATION', None)
        else:
            os.environ['RMW_IMPLEMENTATION'] = self.old_rmw
        if self.old_cyclone_uri is not None:
            os.environ['CYCLONEDDS_URI'] = self.old_cyclone_uri


@pytest.fixture
def rig(tmp_path):
    simulation = Rig(tmp_path)
    try:
        yield simulation
    finally:
        simulation.close()


def test_unprotected_sink_sigkill_leaves_persistent_controller_moving(rig):
    rig.command = 0.2
    sink = rig.start('sink', protected=False)
    rig.wait(lambda: rig.target == 0.2)
    killed = rig.kill(sink)
    rig.spin(0.8)
    assert rig.target == 0.2
    assert not [r for r in rig.requests if r[0] > killed + 0.05]


def test_sink_sigkill_stops_and_restart_requires_current_operator_release(rig):
    rig.command = 0.2
    rig.start('guard')
    sink = rig.start('sink')
    rig.enable()
    killed = rig.kill(sink)
    rig.wait(lambda: rig.target == 0.0, timeout=1.0)
    rig.spin(0.25)
    after = [r for r in rig.requests if r[0] > killed]
    first_stop = next(r[0] for r in after if r[1] == 1003)
    assert first_stop - killed < 0.5
    assert len([r for r in after if r[1] == 1003 and r[2] < 0]) >= 2
    assert not [r for r in after if r[0] > first_stop and r[1] == 1008]
    assert rig.faults and rig.guard_status['latched']
    # Restarted sink Move, a periodic released status, and old release alone
    # must not release the guard. Fault delivery is disabled for this probe.
    rig.auto_protect = False
    old_generation = rig.arbiter_generation
    old_intent_id = max(request_id for _, request_id in rig.intents)
    rig.start('sink')
    rig.wait(lambda: any(
        api == 1008 and request_id > old_intent_id
        for api, request_id in rig.intents), timeout=10.0)
    assert rig.target == 0.0 and rig.guard_status['latched']
    rig.arbiter_generation = old_generation - 1
    rig.release(guard_authorize=False)
    rig.spin(0.2)
    assert rig.target == 0.0 and rig.guard_status['latched']
    rig.arbiter_generation = old_generation
    rig.protect()
    rig.wait(lambda: not rig.guard_status['fault_required'])
    rig.spin(0.1)
    rig.release()
    rig.wait(lambda: rig.target == 0.2)


def test_guard_restarts_keep_latch_and_guard_death_leaves_gap(rig):
    rig.command = 0.2
    guard = rig.start('guard')
    rig.start('sink')
    rig.enable()
    # Losing the guard itself leaves no independent final stop sender.
    killed = rig.kill(guard)
    rig.spin(0.5)
    assert rig.target == 0.2
    assert not [r for r in rig.requests if r[0] > killed + 0.05]
    rig.auto_protect = False
    for _ in range(3):
        rig.guard_status = None
        guard = rig.start('guard')
        rig.wait(lambda: rig.guard_status and rig.guard_status['latched'])
        rig.wait(lambda: rig.target == 0.0)
        rig.spin(0.15)
        assert rig.guard_status['latched']
        rig.kill(guard)
    rig.guard_status = None
    rig.start('guard')
    rig.protect()
    rig.wait(lambda: rig.guard_status
             and not rig.guard_status['fault_required'])
    rig.spin(0.1)
    rig.release()
    rig.wait(lambda: rig.target == 0.2)


def test_duplicate_intents_cannot_extend_move_watchdog(rig):
    rig.command = 0.2
    rig.start('guard')
    sink = rig.start('sink')
    rig.enable()
    rig.kill(sink)
    duplicate = rig.Request()
    duplicate.header.identity.id = rig.intents[-1][1]
    duplicate.header.identity.api_id = 1008
    duplicate.parameter = '{"x":0.2,"y":0,"z":0}'
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        rig.intent_pub.publish(duplicate)
        rig.spin(0.02)
    assert rig.target == 0.0 and rig.guard_status['latched']


@pytest.mark.parametrize('policy', ['deny', 'drop'])
def test_denied_or_lost_stop_remains_unproven(rig, policy):
    rig.command = 0.2
    rig.stop_policy = policy
    rig.start('guard')
    sink = rig.start('sink')
    rig.enable()
    killed = rig.kill(sink)
    rig.wait(lambda: rig.guard_status['latched'])
    rig.spin(0.3)
    stops = [r for r in rig.requests if r[0] > killed and r[1] == 1003]
    assert len(stops) >= 2
    assert rig.target == 0.2
    assert rig.guard_status['stop_acked'] is False
    assert rig.guard_status['measured_stopped'] is False


def test_forwarded_stop_identity_preserves_existing_sink_ack_chain(rig):
    rig.start('guard')
    rig.start('sink')
    rig.wait(lambda: rig.sink_status and rig.sink_status.stop_acked)
    request_id = rig.sink_status.last_request_id
    assert request_id > 0
    assert any(r[1] == 1003 and r[2] == request_id for r in rig.requests)
    assert (1003, request_id, 0) in rig.responses
    rig.wait(lambda: any(r[1] == 1003 and r[2] < 0 for r in rig.requests))
    assert any(r[1] == 1003 and r[2] < 0 for r in rig.requests)
    assert rig.sink_status.stopped is False


def test_guard_dry_run_and_arming_boundaries(rig):
    rig.status_streaming = False
    guard = rig.start('guard', extra={'mode': 'dry_run'})
    rig.wait(lambda: rig.guard_status is not None)
    rig.wait(lambda: rig.faults)
    assert not rig.requests
    assert rig.faults and not rig.native_faults
    rig.kill(guard)
    unsafe_topic = rig.start(
        'guard', extra={'request_topic': '/api/sport/request'})
    assert unsafe_topic.wait(timeout=5) != 0
    nonlocal_guard = rig.start(
        'guard', env=dict(rig.env, ROS_LOCALHOST_ONLY='0'))
    assert nonlocal_guard.wait(timeout=5) != 0
    assert not rig.requests


def test_fake_go2_without_expiry_keeps_moving_after_unprotected_sink_kill(rig):
    rig.start_fake_go2()
    rig.command = 0.2
    sink = rig.start('sink', protected=False)
    rig.wait(lambda: rig.odom and rig.odom[-1][1] > 0.15)
    killed = rig.kill(sink)
    rig.spin(1.3)
    late_velocity = [velocity for stamp, velocity in rig.odom
                     if stamp > killed + 1.1]
    assert late_velocity and min(late_velocity) > 0.15
    assert not [r for r in rig.requests if r[0] > killed + 0.05]


def test_fake_go2_guard_stops_after_sink_kill_and_keeps_latch(rig):
    rig.start_fake_go2()
    rig.command = 0.2
    rig.start('guard')
    sink = rig.start('sink')
    rig.wait(lambda: rig.guard_status and rig.guard_status['latched']
             and not rig.guard_status['fault_required'] and rig.intents)
    rig.release()
    rig.wait(lambda: rig.odom and rig.odom[-1][1] > 0.15)
    killed = rig.kill(sink)
    rig.wait(lambda: rig.guard_status['latched'])
    rig.wait(lambda: rig.odom[-1][1] < 0.01, timeout=1.5)
    rig.spin(max(0.3, killed + 1.3 - time.monotonic()))
    stops = [r for r in rig.requests if r[0] > killed and r[1] == 1003]
    assert len(stops) >= 2
    first_stop = stops[0][0]
    assert first_stop - killed < 0.5
    assert not [r for r in rig.requests
                if r[0] > first_stop and r[1] == 1008]
    assert rig.guard_status['latched']
    assert rig.arbiter_protected
    assert max(velocity for stamp, velocity in rig.odom
               if stamp > killed + 1.0) < 0.01
    # A running sink may resume publishing, but cannot release either latch.
    rig.start('sink')
    rig.spin(0.6)
    assert rig.guard_status['latched'] and rig.arbiter_protected
    assert rig.odom[-1][1] < 0.01


def test_replayed_arbiter_pair_and_old_guard_token_cannot_open_restart(rig):
    rig.command = 0.2
    old_guard = rig.start('guard')
    rig.start('sink')
    rig.enable()
    old_token = rig.guard_status['release_token']
    rig.kill(old_guard)
    rig.auto_protect = False
    rig.status_streaming = False
    rig.guard_status = None
    rig.start('guard')
    rig.wait(lambda: rig.guard_status and rig.guard_status['latched'])
    rig.wait(lambda: rig.target == 0.0)
    startup_token = rig.guard_status['release_token']
    assert rig.guard_status['release_token'] != old_token

    # Replay the old arbiter's held/released pair with apparently ordered
    # status and a live sink. It can establish readiness, never authorization.
    status = rig.ArbiterStatus()
    status.seq = 500
    status.release_challenge = 'test-arbiter:1'
    status.protective_latched = True
    status.hold_active = True
    rig.arbiter_pub.publish(status)
    rig.wait(lambda: not rig.guard_status['fault_required'])
    rig.spin(0.06)
    rig.arbiter_seq = 500
    rig.arbiter_generation = 1
    rig.arbiter_protected = False
    rig.status_streaming = True
    rig.wait(lambda: rig.guard_status['release_ready'])
    assert rig.guard_status['release_token'] != startup_token
    rig.spin(0.1)
    assert rig.guard_status['latched'] and rig.target == 0.0
    rig.release_guard(old_token)
    rig.spin(0.1)
    assert rig.guard_status['latched'] and rig.target == 0.0
    # A same-instance token captured before health cannot arrive late and open.
    rig.release_guard(startup_token)
    rig.spin(0.1)
    assert rig.guard_status['latched'] and rig.target == 0.0
    rig.release_guard()
    rig.wait(lambda: rig.target == 0.2)
