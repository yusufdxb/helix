#!/usr/bin/env python3
"""Exercise the complete native HELIX stack and Sport gate, localhost only.

Requires the vendor ROS messages and built native packages. Starts the fixed
helix_native_guard_sim launch, never the hardware launch. Four test-owned ROS
relays inject communication failures without changing network configuration.
FakeGo2 command expiry is disabled. All recorded times are host monotonic
receipt times, not controller execution timestamps. This is simulation evidence.

Run after sourcing install/setup.bash:
    python3 scripts/sim_native_sport_gate.py --out results/native_sport_gate_integration_sim.json
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import signal
import subprocess
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

MOVE, STOP = 1008, 1003
BASE = '/helix/experimental/native/'
CASES = ('sink', 'supervisor', 'arbiter', 'guard', 'intent_loss', 'status_loss',
         'request_loss', 'response_loss', 'state_loss', 'reject_stop', 'ack_only_stop',
         'drop_stop')
TARGETS = {'sink': 'helix_go2_sport_sink', 'supervisor': 'helix_supervisor',
           'arbiter': 'helix_arbiter', 'guard': 'helix_sport_guard'}


def native_profile():
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('helix_bringup')) / 'launch/helix_native.launch.py'
    spec = importlib.util.spec_from_file_location('native_gate_profile', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pids(executable):
    """Only processes belonging to this test run, never another ROS domain."""
    result = []
    marker = ('HELIX_NATIVE_GATE_SIM=' + os.environ['HELIX_NATIVE_GATE_SIM']).encode()
    for path in Path('/proc').iterdir():
        if not path.name.isdecimal():
            continue
        try:
            cmd = (path / 'cmdline').read_bytes().split(b'\0')[0].decode()
            env = (path / 'environ').read_bytes().split(b'\0')
            if cmd.endswith('/' + executable) and marker in env:
                result.append(int(path.name))
        except (OSError, UnicodeError):
            pass
    return result


def restart_argv(name, directory):
    import yaml
    from ament_index_python.packages import get_package_prefix
    spec = next(s for s in native_profile().guard_sim_node_specs() if s['name'] == name)
    argv = [str(Path(get_package_prefix(spec['package'])) / 'lib' /
                spec['package'] / spec['executable']), '--ros-args', '-r', '__node:=' + name]
    for i, parameter in enumerate(spec['parameters']):
        if isinstance(parameter, dict):
            path = directory / f'{name}_{i}.yaml'
            path.write_text(yaml.safe_dump({name: {'ros__parameters': parameter}}))
        else:
            path = Path(parameter)
        argv += ['--params-file', str(path)]
    for source, target in spec['remappings']:
        argv += ['-r', source + ':=' + target]
    return argv, spec['lifecycle']


class Recorder:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from std_msgs.msg import String
        from unitree_api.msg import Request, Response

        from helix_msgs.msg import (
            ArbiterStatus,
            FaultEvent,
            SinkStatus,
            SupervisorStatus,
        )
        from helix_msgs.srv import ReleaseHold

        self.ros = rclpy
        rclpy.init()
        self.node = Node('native_gate_test_driver')
        self.start = time.monotonic()
        self.events = []
        self.guard = self.arbiter = self.supervisor = self.feedback = None
        self.supervisor_at = None
        self.speed = None
        self.still_at = None
        self.state_keys = {}
        self.links = dict(intent=True, status=True, request=True, response=True)
        self.authorized = False
        self.stale_tokens = []
        self.violations = []
        self.ReleaseHold = ReleaseHold
        self.String = String
        self.client = self.node.create_client(ReleaseHold, BASE + 'supervisor/release')
        self.release_pub = self.node.create_publisher(String, BASE + 'sport/release', 10)
        self.nav = self.node.create_publisher(Twist, BASE + 'nav/cmd_vel', 10)
        command = Twist()
        command.linear.x = 0.2
        self.node.create_timer(0.05, lambda: self.nav.publish(command))
        for link, cls, source, target in (
            ('intent', Request, 'sport/intent_raw', 'sport/intent'),
            ('status', ArbiterStatus, 'arbiter/status_raw', 'arbiter/status'),
            ('request', Request, 'controller/request_raw', 'controller/request'),
            ('response', Response, 'controller/response_raw', 'controller/response'),
        ):
            pub = self.node.create_publisher(cls, BASE + target, 100)
            self.node.create_subscription(
                cls, BASE + source,
                lambda m, link=link, pub=pub: self.relay(link, pub, m), 100)
        self.node.create_subscription(String, BASE + 'sport/guard_status', self.on_guard, 100)
        self.node.create_subscription(SupervisorStatus, BASE + 'supervisor/status', self.on_supervisor, 100)
        self.node.create_subscription(SinkStatus, BASE + 'sport/feedback', self.on_feedback, 100)
        self.node.create_subscription(Odometry, BASE + 'utlidar/robot_odom', self.on_odom, 100)
        self.node.create_subscription(FaultEvent, BASE + 'faults', lambda m: self.change(
            'fault:' + m.node_name + ':' + m.fault_type, (m.detail,),
            node=m.node_name, type=m.fault_type, detail=m.detail), 100)

    def now(self):
        return time.monotonic() - self.start

    def event(self, kind, **kw):
        e = dict(t=round(self.now(), 6), kind=kind, **kw)
        self.events.append(e)
        return e['t']

    def change(self, kind, key, **kw):
        if self.state_keys.get(kind) != key:
            self.state_keys[kind] = key
            self.event(kind, **kw)

    def relay(self, link, pub, m):
        if link == 'status':
            self.arbiter = m
            self.change('arbiter', (m.protective_latched, m.hold_active, m.release_challenge,
                                   m.supervisor_epoch), latched=m.protective_latched,
                        hold=m.hold_active, challenge=m.release_challenge, epoch=m.supervisor_epoch)
        else:
            fields = dict(api=m.header.identity.api_id, id=m.header.identity.id)
            if link == 'response':
                fields['code'] = m.header.status.code
            self.event(link, delivered=self.links[link], **fields)
            if link == 'request' and fields['api'] == MOVE and not self.authorized:
                self.violations.append(self.event('unauthorized_move', id=fields['id']))
        if self.links[link]:
            pub.publish(m)

    def on_guard(self, m):
        self.guard = json.loads(m.data)
        g = self.guard
        self.change('guard', (g['latched'], g['reason'], g['release_ready'], g['release_token'],
                              g['output_healthy']), **g)

    def on_supervisor(self, m):
        self.supervisor = m
        self.supervisor_at = self.now()
        self.change('supervisor', (m.state, tuple(m.missing_nodes)), state=m.state,
                    reason=m.reason, missing=list(m.missing_nodes))

    def on_feedback(self, m):
        self.feedback = m
        self.change('feedback', (m.stop_acked, m.stopped, m.feedback_available),
                    stop_acked=m.stop_acked, stopped=m.stopped,
                    available=m.feedback_available)

    def on_odom(self, m):
        self.speed = abs(m.twist.twist.linear.x)
        if self.speed < 0.01:
            if self.still_at is None:
                self.still_at = self.now()
        else:
            self.still_at = None

    def spin(self, seconds):
        self.until(lambda: False, seconds)

    def until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.ros.spin_once(self.node, timeout_sec=0.01)
            if predicate():
                return True
        return False

    def authorization(self):
        return self.ReleaseHold.Request(supervisor_epoch=self.arbiter.supervisor_epoch,
                                        supervisor_revision=self.supervisor.revision,
                                        release_challenge=self.arbiter.release_challenge)

    def release_native(self, auth=None):
        if not self.client.wait_for_service(timeout_sec=2):
            return False
        future = self.client.call_async(auth or self.authorization())
        ok = self.until(future.done, 3)
        result = future.result() if ok else None
        self.event('operator_native', accepted=bool(result and result.success),
                   message=result.message if result else 'no response')
        return bool(result and result.success)

    def release_guard(self, token=None):
        self.event('operator_guard', stale=token is not None)
        self.release_pub.publish(self.String(data=token or self.guard['release_token']))

    def moving(self):
        return bool(self.guard and not self.guard['latched'] and self.speed and self.speed > .15)

    def operator_resume(self):
        # These are simulated, separate operator actions. During startup an
        # expired pre-controller request may require a new protective generation
        # before the guard can observe the held/released handshake. Runtime
        # nodes never perform this test-driver retry or grant authorization.
        for attempt in range(3):
            self.event('operator_snapshot', attempt=attempt,
                       challenge=self.arbiter.release_challenge,
                       revision=self.supervisor.revision)
            if not self.release_native():
                return False
            if self.until(lambda: self.guard['release_ready'], 1):
                break
            self.spin(1)
        else:
            return False
        for token in self.stale_tokens:
            self.release_guard(token)
        if self.stale_tokens:
            self.spin(.2)
            if not self.guard['latched']:
                return False
        self.authorized = True
        self.release_guard()
        return self.until(self.moving, 5)

    def set_fake(self, **values):
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        client = self.node.create_client(SetParameters, '/fake_go2/set_parameters')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('fake parameters unavailable')
        future = client.call_async(SetParameters.Request(parameters=[
            Parameter(k, value=v).to_parameter_msg() for k, v in values.items()]))
        if not self.until(future.done, 3) or not all(r.successful for r in future.result().results):
            raise RuntimeError('fake parameter change failed')

    def lifecycle_up(self, name):
        from lifecycle_msgs.msg import Transition
        from lifecycle_msgs.srv import ChangeState, GetState
        probe = self.node.create_client(GetState, '/' + name + '/get_state')
        change = self.node.create_client(ChangeState, '/' + name + '/change_state')
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if not probe.service_is_ready():
                self.spin(.1)
                continue
            future = probe.call_async(GetState.Request())
            if not self.until(future.done, 1):
                continue
            state = future.result().current_state.id
            if state == 3:
                return
            tid = (Transition.TRANSITION_CONFIGURE if state == 1
                   else Transition.TRANSITION_ACTIVATE)
            future = change.call_async(ChangeState.Request(transition=Transition(id=tid)))
            self.until(future.done, 2)
            self.spin(.1)
        raise RuntimeError('lifecycle restart did not activate: ' + name)

    def close(self):
        self.node.destroy_node()
        self.ros.shutdown()


def run_case(case, directory, restart_count=2):
    rec = Recorder()
    processes = []
    checks = {}
    out = dict(case=case, checks=checks)
    def start(argv, label):
        with open(directory / (label + '.log'), 'w') as log:
            p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p
    def require(key, condition):
        checks[key] = bool(condition)
        if not condition:
            raise RuntimeError(key)
    try:
        start(['ros2', 'launch', 'helix_bringup', 'helix_native_guard_sim.launch.py'], 'launch')
        require('startup_healthy', rec.until(
            lambda: rec.guard and rec.guard['output_healthy'] and rec.arbiter and
            rec.arbiter.supervisor_epoch and rec.supervisor and not rec.supervisor.missing_nodes, 35))
        rec.spin(1)
        require('startup_no_move', not any(e['kind'] == 'request' and e['api'] == MOVE
                                         for e in rec.events))
        old_token = rec.guard['release_token']
        require('startup_both_latched', rec.guard['latched'] and rec.arbiter.protective_latched)
        require('initial_operator_release', rec.operator_resume())
        rec.spin(.5)
        old_auth = rec.authorization()
        old_ready_token = rec.guard['release_token']
        rec.stale_tokens = [old_token, old_ready_token]
        if case in ('reject_stop', 'ack_only_stop', 'drop_stop'):
            rec.set_fake(stop_policy={'reject_stop': 'reject', 'ack_only_stop': 'ack_only',
                                      'drop_stop': 'drop'}[case])
        inject = rec.event('injection', case=case)
        if case in TARGETS:
            found = pids(TARGETS[case])
            require('one_target', len(found) == 1)
            os.kill(found[0], signal.SIGKILL)
        elif case == 'state_loss':
            rec.set_fake(state_feedback_enabled=False)
        elif case.endswith('_stop'):
            rec.links['intent'] = False
        else:
            rec.links[case.removesuffix('_loss')] = False
        rec.spin(5.5)
        after = [e for e in rec.events if e['t'] >= inject]
        detections = [e for e in after if
                      (e['kind'] == 'guard' and e['latched']) or
                      (e['kind'] == 'arbiter' and (e['latched'] or e['hold'])) or
                      (e['kind'] == 'supervisor' and e['state'] not in ('NOMINAL', 'STARTING'))]
        require('failure_detected', bool(detections))
        detected = min(e.get('detected_at', 0) - rec.start
                       if e.get('detected_at', 0) - rec.start >= inject else e['t']
                       for e in detections)
        stop_pub = next((e for e in after if e['kind'] == 'request' and e['api'] == STOP), None)
        stop_delivery = next((e for e in after if e['kind'] == 'request' and e['api'] == STOP
                              and e['delivered']), None)
        stop_ack = next((e for e in after if e['kind'] == 'response' and e['api'] == STOP
                         and e['code'] == 0 and e['delivered']), None)
        still = rec.still_at if rec.still_at and rec.now() - rec.still_at >= .2 else None
        out['latency_ms'] = dict(injection_to_detection=round(1000 * (detected-inject), 3),
            detection_to_stop_publication=None if not stop_pub else round(1000*(stop_pub['t']-detected), 3),
            detection_to_stop_delivery=None if not stop_delivery else round(1000*(stop_delivery['t']-detected), 3),
            detection_to_stop_ack=None if not stop_ack else round(1000*(stop_ack['t']-detected), 3),
            detection_to_simulated_still=None if not still else round(1000*(still-detected), 3))
        # Different topics have independent delivery queues. Allow requests
        # already queued before detection, but never a Move after first Stop.
        boundary = stop_pub['t'] if stop_pub else detected + .1
        require('no_move_after_stop_boundary', not any(
            e['kind'] == 'request' and e['api'] == MOVE and e['t'] > boundary for e in after))
        persists = case in ('guard', 'request_loss', 'reject_stop', 'ack_only_stop', 'drop_stop')
        if persists:
            require('unresolved_motion_exposed', rec.speed > .15 and still is None)
            require('no_false_stop_confirmation', rec.supervisor.state != 'STOP_CONFIRMED')
        else:
            require('simulated_stop', still is not None)
            require('stop_delivered', stop_delivery is not None)
        if case in ('response_loss', 'reject_stop', 'ack_only_stop', 'drop_stop', 'state_loss', 'guard'):
            require('stop_unconfirmed', rec.supervisor.state == 'STOP_UNCONFIRMED')
        out['failure_state'] = (rec.supervisor.state if rec.supervisor_at is not None and
                                rec.now() - rec.supervisor_at < 1 else 'UNAVAILABLE')
        out['speed_before_repair'] = rec.speed
        rec.authorized = False
        if case in TARGETS:
            name = TARGETS[case]
            for index in range(restart_count):
                argv, lifecycle = restart_argv(name, directory)
                start(argv, f'restart_{index}')
                if lifecycle:
                    rec.lifecycle_up(name)
                rec.spin(1.2)
                if index < restart_count - 1:
                    found = pids(name)
                    require(f'restart_{index}_one_target', len(found) == 1)
                    os.kill(found[0], signal.SIGKILL)
                    rec.spin(.8)
        else:
            rec.links = {key: True for key in rec.links}
            rec.set_fake(stop_policy='accept', state_feedback_enabled=True)
        require('recovered_output_health', rec.until(
            lambda: rec.guard and rec.guard['output_healthy'] and
            not rec.supervisor.missing_nodes, 20))
        rec.spin(3)
        require('health_does_not_release', rec.guard['latched'] and rec.arbiter.protective_latched)
        require('old_native_release_rejected', not rec.release_native(old_auth))
        rec.release_guard(old_token)
        rec.release_guard(old_ready_token)
        rec.spin(.5)
        require('stale_guard_release_rejected', rec.guard['latched'])
        require('current_operator_release', rec.operator_resume())
        rec.spin(.5)
        require('no_unauthorized_moves', not rec.violations)
        out['restarts'] = restart_count if case in TARGETS else 0
    except Exception as error:
        out['error'] = str(error)
    finally:
        out['events'] = rec.events
        out['passed'] = bool(checks) and all(checks.values()) and 'error' not in out
        for p in reversed(processes):
            if p.poll() is None:
                # ros2 launch forwards the signal to its children. Signalling
                # its whole group as well delivers SIGINT twice during cleanup.
                p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        rec.close()
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=CASES)
    parser.add_argument('--restarts', type=int, default=2)
    parser.add_argument('--domain', type=int, default=218)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232 or args.restarts < 1:
        parser.error('requires isolated domain 1..232 and at least one restart')
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=str(args.domain),
                      RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      HELIX_NATIVE_GATE_SIM=str(os.getpid()) + ':' + str(time.monotonic_ns()))
    os.environ['CYCLONEDDS_URI'] = native_profile().GUARD_SIM_CYCLONE_URI
    directory = Path(tempfile.mkdtemp(prefix='helix_native_gate_'))
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros_logs')
    output = dict(evidence_class='Sim', controller='FakeGo2, Move expiry disabled',
                  timestamp=datetime.now(timezone.utc).isoformat(),
                  hardware_motion=False, controller_mode_changes=False,
                  timing='host monotonic receipt; still below 0.01 m/s for 0.2 s', cases=[])
    for case in args.cases:
        case_dir = directory / case
        case_dir.mkdir()
        result = run_case(case, case_dir, args.restarts)
        output['cases'].append(result)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(output, indent=2) + '\n')
        print(case, 'PASS' if result['passed'] else 'FAIL', result.get('error', ''), flush=True)
    print('Local diagnostic logs:', directory)
    return 0 if all(c['passed'] for c in output['cases']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
