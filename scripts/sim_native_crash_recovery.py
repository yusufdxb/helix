#!/usr/bin/env python3
"""Crash-recovery of the armed native stack against the fake GO2. Simulation only.

Each case starts a fresh ``helix_native.launch.py sink_mode:=armed
actuation_enabled:=true`` stack and ``helix_fake_go2`` on ROS_LOCALHOST_ONLY
with its own domain id, streams vx 0.2 on the nav source until the sink is
sending Move (1008) and the supervisor is NOMINAL, then SIGKILLs one node:

    detector    helix_anomaly_detector: a critical expected node. Expect a
                CRASH hold, StopMove (1003), no Move while held; after a
                restart AWAIT_RELEASE (auto_resume is false), no Move; after
                the operator release NOMINAL and Move again.
    supervisor  helix_supervisor: the hold stream stops. Expect the arbiter to
                fail closed (hold_timeout_sec) and the sink to send StopMove.
                The supervisor is then restarted and what follows is recorded.
    arbiter     helix_arbiter: the sink input stops. Expect the sink deadman
                (input_timeout_sec) to send StopMove.
    sink        helix_go2_sport_sink: the only /api/sport/request publisher.
                Nothing is left to send StopMove; recorded to show the gap.

Every request and response, supervisor status, hold and fault is recorded
with a monotonic stamp; the case verdict is computed from those records. The
fake GO2 is a first-order stand-in, so nothing here is evidence about the
robot. Output: one JSON file with every case.

Usage:
    source /opt/ros/humble/setup.bash
    source <unitree_ros2>/cyclonedds_ws/install/setup.bash
    source install/setup.bash
    python3 scripts/sim_native_crash_recovery.py --out results/native_crash_recovery_sim.json
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
MOVE, STOP = 1008, 1003
CASES = ('detector', 'supervisor', 'arbiter', 'sink')
TARGETS = {'detector': 'lib/helix_sensing_cpp/helix_anomaly_detector',
           'supervisor': 'lib/helix_supervisor_cpp/helix_supervisor',
           'arbiter': 'lib/helix_arbiter_cpp/helix_arbiter',
           'sink': 'lib/helix_go2_sink_cpp/helix_go2_sport_sink'}


def _pids(fragment: str) -> list[int]:
    out = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            cmd = Path(f'/proc/{entry}/cmdline').read_bytes().split(b'\0')[0].decode()
        except OSError:
            continue
        if cmd.endswith(fragment):
            out.append(int(entry))
    return out


def _share(pkg: str) -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(pkg))


def _restart_argv(case: str, tmp: Path) -> list[str]:
    """The killed node's executable with the parameters helix_native passes."""
    import yaml
    from ament_index_python.packages import get_package_prefix
    bringup = _share('helix_bringup')
    spec = importlib.util.spec_from_file_location(
        'native_launch', bringup / 'launch' / 'helix_native.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if case == 'detector':
        node, pkg = 'helix_anomaly_detector', 'helix_sensing_cpp'
        files = [str(bringup / 'config' / 'helix_params.yaml')]
        extra = dict(mod.DETECTOR_OVERRIDES)
    else:
        node, pkg = 'helix_supervisor', 'helix_supervisor_cpp'
        files = []
        extra = {'expected_nodes': mod.EXPECTED_NODES, 'critical_nodes': ['*'],
                 'actuation_enabled': True, 'auto_resume': False}
    f = tmp / f'{node}.yaml'
    f.write_text(yaml.safe_dump({node: {'ros__parameters': extra}}))
    argv = [str(Path(get_package_prefix(pkg)) / 'lib' / pkg / node), '--ros-args',
            '-r', f'__node:={node}']
    for p in files + [str(f)]:
        argv += ['--params-file', p]
    return argv


class Recorder:

    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_srvs.srv import Trigger
        from unitree_api.msg import Request, Response

        from helix_msgs.msg import FaultEvent, HelixHold, SupervisorStatus

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node('crash_recovery_recorder')
        self.events: list[dict] = []
        self.t0 = time.monotonic()
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state = None
        self._last_hold = None
        self.node.create_subscription(Request, '/api/sport/request', lambda m: self.ev(
            'request', api=m.header.identity.api_id, id=m.header.identity.id), rel)
        self.node.create_subscription(Response, '/api/sport/response', lambda m: self.ev(
            'response', api=m.header.identity.api_id, id=m.header.identity.id,
            code=m.header.status.code), rel)
        self.node.create_subscription(SupervisorStatus, '/helix/supervisor/status',
                                      self._status, rel)
        self.node.create_subscription(HelixHold, '/helix/hold', lambda m: self.ev(
            'hold', hold=m.hold, epoch=m.epoch, fault=m.fault_id), be)
        self.node.create_subscription(FaultEvent, '/helix/faults', lambda m: self.ev(
            'fault', type=m.fault_type, node=m.node_name, detail=m.detail[:120]), rel)
        self.nav = self.node.create_publisher(Twist, '/helix_hw/nav/cmd_vel', rel)
        self.twist = Twist()
        self.streaming = False
        self.node.create_timer(0.05, self._nav)
        self.release = self.node.create_client(Trigger, '/helix/supervisor/release')
        self.Trigger = Trigger

    def now(self) -> float:
        return time.monotonic() - self.t0

    def ev(self, kind: str, **kw) -> None:
        if kind == 'hold':
            key = (kw['hold'], kw['epoch'])
            if key == self._last_hold:
                return  # keep hold changes only
            self._last_hold = key
        self.events.append({'t': round(self.now(), 4), 'kind': kind, **kw})

    def _status(self, m) -> None:
        if m.state != self.state:
            self.state = m.state
            self.ev('state', state=m.state, hold=m.hold, reason=m.reason[:120],
                    missing=list(m.missing_nodes), faults=list(m.active_faults)[:4])

    def _nav(self) -> None:
        if self.streaming:
            self.twist.linear.x = 0.2
            self.nav.publish(self.twist)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def spin_until(self, pred, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            if pred():
                return True
        return False

    def moves_since(self, t: float) -> int:
        return sum(1 for e in self.events
                   if e['kind'] == 'request' and e['api'] == MOVE and e['t'] > t)

    def call_release(self) -> dict:
        if not self.release.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'message': 'no release service'}
        fut = self.release.call_async(self.Trigger.Request())
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        r = fut.result()
        return {'ok': bool(r and r.success), 'message': r.message if r else 'timeout'}

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()


def _lifecycle_up(rec: Recorder, name: str) -> str:
    """Configure then activate ``name``; '' on success, else the failed step."""
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState
    # A SIGKILLed node's services stay in DDS discovery until its lease runs
    # out, so "service ready" can mean the dead process. Wait until a
    # get_state request is actually answered: only the new process can.
    probe = rec.node.create_client(GetState, f'/{name}/get_state')
    end = time.monotonic() + 20.0
    while True:
        if time.monotonic() > end:
            return 'get_state never answered'
        if not probe.service_is_ready():
            rec.rclpy.spin_once(rec.node, timeout_sec=0.05)
            continue
        fut = probe.call_async(GetState.Request())
        if rec.spin_until(fut.done, 1.0):
            break
    cli = rec.node.create_client(ChangeState, f'/{name}/change_state')
    while not cli.service_is_ready():
        rec.rclpy.spin_once(rec.node, timeout_sec=0.05)
        if time.monotonic() > end:
            return 'no change_state service'
    for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        fut = cli.call_async(ChangeState.Request(transition=Transition(id=tid)))
        rec.spin_until(fut.done, 10.0)
        if not fut.done():
            return f'transition {tid} timed out'
        if not fut.result().success:
            return f'transition {tid} refused'
    return ''


def run_case(case: str, log_dir: Path) -> dict:
    env = dict(os.environ)
    share = _share('helix_arbiter')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'helix_bringup', 'helix_native.launch.py', 'sink_mode:=armed',
         'actuation_enabled:=true', f'arbiter_config:={share}/config/arbiter_hw.yaml',
         'cmd_vel_out:=/helix_hw/cmd_vel'],
        env=env, stdout=open(log_dir / f'{case}_launch.log', 'w'), stderr=subprocess.STDOUT,
        start_new_session=True)
    fake = subprocess.Popen(['ros2', 'run', 'helix_arbiter', 'helix_fake_go2'], env=env,
                            stdout=open(log_dir / f'{case}_fake.log', 'w'),
                            stderr=subprocess.STDOUT, start_new_session=True)
    restarted = None
    rec = Recorder()
    out = {'case': case, 'checks': {}, 'notes': []}
    try:
        ok = rec.spin_until(lambda: rec.state == 'NOMINAL', 40.0)
        out['checks']['reached_nominal'] = ok
        if not ok:
            return out
        rec.streaming = True
        t_stream = rec.now()
        out['checks']['moving_before_kill'] = rec.spin_until(
            lambda: rec.moves_since(t_stream) >= 20, 10.0)
        rec.spin_for(1.0)
        pids = _pids(TARGETS[case])
        out['checks']['one_target_process'] = len(pids) == 1
        if len(pids) != 1:
            return out
        t_kill = rec.now()
        os.kill(pids[0], signal.SIGKILL)
        rec.ev('kill', target=case, pid=pids[0])
        rec.spin_for(3.0)
        after = [e for e in rec.events if e['t'] > t_kill and e['kind'] == 'request']
        stops = [e for e in after if e['api'] == STOP]
        out['first_stop_after_kill_ms'] = (round(1000 * (stops[0]['t'] - t_kill), 1)
                                           if stops else None)
        last_move = max((e['t'] for e in after if e['api'] == MOVE), default=None)
        out['last_move_after_kill_ms'] = (round(1000 * (last_move - t_kill), 1)
                                          if last_move is not None else None)
        out['moves_after_first_stop'] = (sum(1 for e in after if e['api'] == MOVE
                                             and e['t'] > stops[0]['t']) if stops else None)
        if case in ('detector', 'supervisor', 'arbiter'):
            out['checks']['stopmove_after_kill'] = bool(stops)
            out['checks']['no_move_after_first_stop'] = out['moves_after_first_stop'] == 0
        else:
            out['checks']['no_request_after_kill'] = not after
            out['notes'].append('nothing publishes /api/sport/request once the sink is gone; '
                                'the robot keeps its last Move until its own timeout')
        if case in ('detector', 'supervisor'):
            with tempfile.TemporaryDirectory() as tmp:
                argv = _restart_argv(case, Path(tmp))
                restarted = subprocess.Popen(
                    argv, env=env, stdout=open(log_dir / f'{case}_restart.log', 'w'),
                    stderr=subprocess.STDOUT, start_new_session=True)
                t_restart = rec.now()
                rec.ev('restart', target=case)
                why = _lifecycle_up(
                    rec, 'helix_anomaly_detector' if case == 'detector' else 'helix_supervisor')
                out['checks']['restart_activated'] = not why
                if why:
                    out['notes'].append(f'restart: {why}')
            if case == 'detector':
                out['checks']['await_release_after_restart'] = rec.spin_until(
                    lambda: rec.state == 'AWAIT_RELEASE', 15.0)
                rec.spin_for(2.0)
                out['checks']['no_move_while_awaiting_release'] = rec.moves_since(t_restart) == 0
                t_rel = rec.now()
                out['release'] = rec.call_release()
                rec.ev('release', **out['release'])
                out['checks']['nominal_after_release'] = rec.spin_until(
                    lambda: rec.state == 'NOMINAL', 5.0)
                out['checks']['move_resumes_after_release'] = rec.spin_until(
                    lambda: rec.moves_since(t_rel) >= 5, 5.0)
            else:
                rec.spin_for(12.0)
                states = [e['state'] for e in rec.events
                          if e['kind'] == 'state' and e['t'] > t_restart]
                out['states_after_restart'] = states
                out['moves_after_restart'] = rec.moves_since(t_restart)
                first_move = next((e['t'] for e in rec.events if e['kind'] == 'request'
                                   and e['api'] == MOVE and e['t'] > t_restart), None)
                out['first_move_after_restart_s'] = (round(first_move - t_restart, 2)
                                                     if first_move else None)
        rec.streaming = False
        rec.spin_for(1.0)
    finally:
        out['events'] = rec.events
        rec.close()
        for p in [launch, fake] + ([restarted] if restarted else []):
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in [launch, fake] + ([restarted] if restarted else []):
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
    out['passed'] = all(out['checks'].values()) and bool(out['checks'])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--cases', nargs='+', default=list(CASES), choices=CASES)
    ap.add_argument('--domain', type=int, default=93)
    ap.add_argument('--out', default=str(REPO / 'results' / 'native_crash_recovery_sim.json'))
    ap.add_argument('--log-dir', default=None)
    args = ap.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION=os.environ.get('RMW_IMPLEMENTATION',
                                                        'rmw_cyclonedds_cpp'))
    os.environ.pop('CYCLONEDDS_URI', None)
    log_dir = Path(args.log_dir or tempfile.mkdtemp(prefix='helix_crash_'))
    log_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for case in args.cases:
        # Each case runs in its own process: one rclpy context per stack.
        r = subprocess.run([sys.executable, __file__, '--one', case, str(log_dir)],
                           capture_output=True, text=True)
        try:
            results.append(json.loads(r.stdout.strip().splitlines()[-1]))
        except (IndexError, json.JSONDecodeError):
            results.append({'case': case, 'passed': False, 'error': r.stderr[-2000:]})
        res = results[-1]
        print(case, 'PASS' if res.get('passed') else 'FAIL', res.get('checks'),
              'stop_ms', res.get('first_stop_after_kill_ms'), flush=True)
        time.sleep(5.0)
    sha = subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--short', 'HEAD'],
                         capture_output=True, text=True).stdout.strip()
    Path(args.out).write_text(json.dumps({
        'generated_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'git_sha': sha, 'evidence_class': 'SIMULATION (fake GO2, localhost only)',
        'cases': results}, indent=1) + '\n')
    return 0 if all(r.get('passed') for r in results if r['case'] != 'sink') else 1


if __name__ == '__main__':
    if len(sys.argv) == 4 and sys.argv[1] == '--one':
        print(json.dumps(run_case(sys.argv[2], Path(sys.argv[3]))))
        sys.exit(0)
    sys.exit(main())
