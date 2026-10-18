#!/usr/bin/env python3
"""Native-runtime stop test on the GO2: fault -> supervisor hold -> robot stop.

Runs on the payload next to helix_native.launch.py, so its time.monotonic()
is the same clock as the supervisor's and the sink's status stamps.

Stages (one per invocation, nothing chains):

  ack     robot still (standing or lying). Inject one benign fault and
          require the full chain HOLD_REQUESTED -> HOLD_APPLIED (arbiter) ->
          STOP_ACKED (StopMove answered with code 0) -> STOP_CONFIRMED
          (measured still), then explicit clear, operator release, NOMINAL.
          Nothing commands motion. Sink must be stop_only or armed.
  motion  robot standing, area clear, operator with the remote. Stream
          vx on the arbiter's nav source until the robot is measured moving,
          inject the fault while the source KEEPS streaming, and require the
          robot to be measured stopped (SportModeState, and the odometry
          topic, which carries the same estimate) with no Move sent while
          held. Then stop streaming,
          clear, release, and require the robot to stay still. Sink armed.

The injected fault is one ANOMALY FaultEvent (severity 2) named
rate_hz/utlidar_helix_injected: it matches the supervisor's stop rule,
touches no sensor and cannot be confused with a real metric. Any other
stop-worthy fault during the stage marks it confounded.

Safety: the runner never sends to /api/sport/request itself; the sink does,
and the sink refuses anything but StopMove 1003 and Move 1008 within its
limits. The runner stops sourcing (the arbiter then outputs zero and the
sink sends StopMove) when stop_guard.OverspeedGuard trips: speed above
--abort-speed for 30 ms, any sample above 0.60 m/s, 0.6 m of travel, or a
silent state topic. Exit code 0 only on PASS.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from stop_guard import OverspeedGuard
from unitree_go.msg import SportModeState, WirelessController

from helix_msgs.msg import ArbiterStatus, FaultEvent, SinkStatus, SupervisorStatus
from helix_msgs.srv import ReleaseHold

INJECTED = 'rate_hz/utlidar_helix_injected'
RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
BEST_EFFORT = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
MOVING = 0.05       # m/s: "robot moving" for the motion stage precondition
STILL = 0.05        # m/s and 0.10 rad/s: measured stop thresholds (sink defaults)
STILL_YAW = 0.10


class Io:
    """Recorder of every relevant topic plus the runner's two publishers."""

    def __init__(self, nav_topic: str) -> None:
        self.node = rclpy.create_node('helix_native_stop_test')
        n = self.node
        self.events = []
        self.sup = None
        self.sink = None
        self.arb = None
        self.speed = math.nan
        self.yaw = math.nan
        self.body_height = math.nan
        self.mode = None
        self.odom_speed = math.nan
        self.guard = None
        self.nav = n.create_publisher(Twist, nav_topic, 10)
        self.faults = n.create_publisher(FaultEvent, '/helix/faults', RELIABLE)
        self.clears = n.create_publisher(FaultEvent, '/helix/fault_clear', RELIABLE)
        n.create_subscription(SupervisorStatus, '/helix/supervisor/status', self._sup, RELIABLE)
        n.create_subscription(SinkStatus, '/helix/sink/status', self._sink, RELIABLE)
        n.create_subscription(ArbiterStatus, '/helix/arbiter/status', self._arb, RELIABLE)
        n.create_subscription(String, '/helix/sink/trace', self._trace, RELIABLE)
        n.create_subscription(FaultEvent, '/helix/faults', self._fault, RELIABLE)
        n.create_subscription(SportModeState, '/sportmodestate', self._state, BEST_EFFORT)
        n.create_subscription(Odometry, '/utlidar/robot_odom', self._odom, BEST_EFFORT)
        # The handheld remote drives the robot without /api/sport/request; it publishes here
        # only while a stick or key is in use (silence is not "remote off").
        n.create_subscription(WirelessController, '/wirelesscontroller', self._remote,
                              BEST_EFFORT)
        self.release_cli = n.create_client(ReleaseHold, '/helix/supervisor/release')

    def ev(self, kind, **kw):
        kw.update(kind=kind, t=time.monotonic())
        self.events.append(kw)

    def _sup(self, m):
        prev = self.sup.state if self.sup else None
        self.sup = m
        if m.state != prev:
            self.ev('sup', state=m.state, hold=m.hold, hold_id=m.hold_id, reason=m.reason,
                    hold_requested_at=m.hold_requested_at, arbiter_ack_at=m.arbiter_ack_at,
                    controller_ack_at=m.controller_ack_at, motion_stopped_at=m.motion_stopped_at,
                    active_faults=list(m.active_faults))

    def _sink(self, m):
        prev = self.sink
        self.sink = m
        if prev is None or (prev.stop_acked, prev.stopped) != (m.stop_acked, m.stopped):
            self.ev('sink', stop_acked=m.stop_acked, stop_ack_sent_at=m.stop_ack_sent_at,
                    stop_acked_at=m.stop_acked_at, stopped=m.stopped,
                    stopped_since=m.stopped_since, last_api_id=m.last_api_id,
                    last_ack_code=m.last_ack_code, mode=m.mode)

    def _arb(self, m):
        prev = self.arb
        self.arb = m
        if prev is None or (prev.reason, prev.hold_fault_id) != (m.reason, m.hold_fault_id):
            self.ev('arb', reason=m.reason, hold_fault_id=m.hold_fault_id,
                    out_vx=m.out_linear_x)

    def _trace(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.ev('req', api_id=d.get('api_id'), reason=d.get('reason'), x=d.get('x'),
                sent=d.get('sent_to_robot'), request_id=d.get('request_id'))

    def _fault(self, m):
        self.ev('fault', node=m.node_name, type=m.fault_type, severity=m.severity)

    def _state(self, m):
        self.speed = math.hypot(m.velocity[0], m.velocity[1])
        if self.guard is not None:
            t = time.monotonic()
            self.guard.speed(t, self.speed, 'state')
            self.guard.position(t, m.position[0], m.position[1])
        self.yaw = m.yaw_speed
        self.body_height = m.body_height
        self.mode = m.mode
        self.ev('state', v=round(self.speed, 4), yaw=round(m.yaw_speed, 4),
                h=round(m.body_height, 4), mode=m.mode)

    def _remote(self, m):
        self.ev('remote', lx=round(m.lx, 3), ly=round(m.ly, 3), rx=round(m.rx, 3),
                ry=round(m.ry, 3), keys=m.keys)

    def _odom(self, m):
        v = m.twist.twist.linear
        self.odom_speed = math.hypot(v.x, v.y)
        if self.guard is not None:
            self.guard.speed(time.monotonic(), self.odom_speed, 'odom')
        self.ev('odom', v=round(self.odom_speed, 4))

    def spin(self, secs, vx=None, abort_speed=0.35, each=None):
        """Spin for secs, streaming vx at 20 Hz if given. True if aborted."""
        end = time.monotonic() + secs
        next_pub = 0.0
        while time.monotonic() < end:
            if vx is not None:
                if self.guard is None:
                    self.guard = OverspeedGuard(abort_speed)
                    self.guard.arm(time.monotonic())
                if self.guard.check_stale(time.monotonic()):
                    self.ev('abort', why=self.guard.reason, v=self.speed, odom=self.odom_speed,
                            guard=dict(self.guard.detail))
                    self.zero()
                    return True
                if time.monotonic() >= next_pub:
                    t = Twist()
                    t.linear.x = float(vx)
                    self.nav.publish(t)
                    next_pub = time.monotonic() + 0.05
            if each is not None:
                each()
            rclpy.spin_once(self.node, timeout_sec=0.005)
        return False

    def zero(self):
        for _ in range(3):
            self.nav.publish(Twist())
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def inject(self):
        f = FaultEvent()
        f.node_name = INJECTED
        f.fault_type = 'ANOMALY'
        f.severity = 2
        f.detail = 'INJECTED by native_stop_test: synthetic benign rate anomaly'
        f.timestamp = time.time()
        f.context_keys = ['metric_name', 'injected']
        f.context_values = [INJECTED, 'true']
        t = time.monotonic()
        self.faults.publish(f)
        self.ev('inject', at=t)
        return t

    def clear(self):
        f = FaultEvent()
        f.node_name = INJECTED
        f.fault_type = 'ANOMALY'
        f.severity = 0
        f.detail = 'INJECTED fault cleared by native_stop_test'
        f.timestamp = time.time()
        f.context_keys = ['metric_name', 'clear_reason']
        f.context_values = [INJECTED, 'recovered']
        t = time.monotonic()
        self.clears.publish(f)
        self.ev('clear', at=t)
        return t

    def release(self):
        if not self.release_cli.wait_for_service(timeout_sec=3.0):
            return False, 'release service unavailable'
        if self.arb is None or self.sup is None:
            return False, 'arbiter or supervisor status unavailable'
        fut = self.release_cli.call_async(ReleaseHold.Request(
            supervisor_epoch=self.arb.supervisor_epoch,
            supervisor_revision=self.sup.revision,
            release_challenge=self.arb.release_challenge))
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        if not fut.done():
            return False, 'release timed out'
        r = fut.result()
        self.ev('release', success=r.success, message=r.message)
        return r.success, r.message

    def wait_for(self, pred, secs, vx=None, abort_speed=0.35):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            if pred():
                return True
            if self.spin(0.02, vx=vx, abort_speed=abort_speed):
                return False
        return pred()

    def param(self, node, name):
        try:
            out = subprocess.run(['ros2', 'param', 'get', node, name], capture_output=True,
                                 text=True, timeout=10).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            return None
        return out


class Stage:

    def __init__(self, io: Io, args) -> None:
        self.io = io
        self.a = args
        self.checks = []

    def check(self, key, what, ok, **detail):
        self.checks.append({'key': key, 'what': what, 'pass': bool(ok), **detail})
        print(f"  [{'PASS' if ok else 'FAIL'}] {what} {detail if detail else ''}")
        return bool(ok)

    def preflight(self, need_modes):
        io = self.io
        io.spin(2.0)
        ok = self.check('sup_live', 'supervisor status received', io.sup is not None)
        ok &= self.check('sink_live', 'sink status received', io.sink is not None)
        ok &= self.check('state_live', 'SportModeState received', not math.isnan(io.speed))
        if not ok:
            return False
        ok &= self.check('nominal', 'supervisor NOMINAL', io.sup.state == 'NOMINAL',
                         state=io.sup.state, faults=list(io.sup.active_faults))
        ok &= self.check('actuation', 'supervisor actuation_enabled', io.sup.actuation_enabled)
        ok &= self.check('sink_mode', f'sink mode in {need_modes}', io.sink.mode in need_modes,
                         mode=io.sink.mode)
        ok &= self.check('feedback', 'sink feedback available', io.sink.feedback_available)
        ok &= self.check('still', 'robot still before the stage',
                         io.speed < STILL and abs(io.yaw) < STILL_YAW, v=io.speed, yaw=io.yaw)
        ok &= self.check('protective_release', 'arbiter explicitly released before stage',
                         io.arb is not None and not io.arb.protective_latched,
                         instruction='operator release required before a new stage')
        return ok

    def chain(self, t_inject, held_deadline=3.0):
        """Wait for the supervisor chain to STOP_CONFIRMED; return its stamps."""
        io = self.io
        reached = io.wait_for(
            lambda: io.sup is not None and io.sup.state in ('STOP_CONFIRMED', 'STOP_UNCONFIRMED')
            and io.sup.hold_id == INJECTED,
            held_deadline, vx=self.stream_vx, abort_speed=self.a.abort_speed)
        s = io.sup
        st = {
            'state': s.state, 'hold': s.hold, 'hold_id': s.hold_id,
            'hold_requested_ms': (s.hold_requested_at - t_inject) * 1e3,
            'arbiter_ack_ms': (s.arbiter_ack_at - t_inject) * 1e3 if s.arbiter_ack_at else None,
            'controller_ack_ms':
                (s.controller_ack_at - t_inject) * 1e3 if s.controller_ack_at else None,
            'motion_stopped_ms':
                (s.motion_stopped_at - t_inject) * 1e3 if s.motion_stopped_at else None,
        }
        self.check('chain', 'supervisor reached STOP_CONFIRMED for the injected fault',
                   reached and s.state == 'STOP_CONFIRMED', **st)
        self.check('arbiter_ack', 'arbiter acknowledged the hold (HELIX_HOLD)',
                   bool(s.arbiter_ack_at))
        self.check('controller_ack', 'robot acknowledged StopMove (code 0)',
                   bool(s.controller_ack_at) and io.sink.stop_acked and io.sink.last_ack_code == 0,
                   last_ack_code=io.sink.last_ack_code)
        return st

    stream_vx = None

    def finish_release(self):
        io = self.io
        self.stream_vx = None
        io.zero()
        io.clear()
        cleared = io.wait_for(lambda: io.sup.state in ('AWAIT_RELEASE', 'NOMINAL'), 6.0)
        self.check('await_release', 'explicit clear -> AWAIT_RELEASE (no auto resume)',
                   cleared and io.sup.state == 'AWAIT_RELEASE', state=io.sup.state)
        ok, msg = io.release()
        self.check('released', 'operator release accepted', ok, message=msg)
        nominal = io.wait_for(lambda: io.sup.state == 'NOMINAL' and not io.sup.hold
                             and io.arb is not None and not io.arb.protective_latched, 3.0)
        self.check('nominal_after', 'supervisor NOMINAL, hold released', nominal,
                   state=io.sup.state)
        t0 = time.monotonic()
        io.spin(3.0)
        moved = [e for e in io.events if e['kind'] == 'state' and e['t'] > t0 and e['v'] > STILL]
        moves = [e for e in io.events
                 if e['kind'] == 'req' and e['t'] > t0 and e['api_id'] == 1008]
        self.check('still_after', 'robot still for 3 s after release (no upstream)',
                   not moved, max_v=max([e['v'] for e in moved], default=0.0))
        self.check('no_move_after', 'no Move sent after release', not moves, n=len(moves))

    def confounded(self, t0):
        real = [e for e in self.io.events if e['kind'] == 'fault' and e['t'] >= t0
                and e['node'] != INJECTED and e['severity'] >= 2
                and e['node'].startswith('rate_hz/utlidar')]
        self.check('unconfounded', 'no real stop-worthy fault during the stage', not real,
                   real=real[:5])
        remote = [e for e in self.io.events if e['kind'] == 'remote' and e['t'] >= t0]
        self.check('no_remote_input', 'no remote stick or key input during the stage',
                   not remote, n=len(remote), first=remote[:3])


def run_ack(stage: Stage) -> None:
    io = stage.io
    if not stage.preflight(('stop_only', 'armed')):
        return
    t0 = time.monotonic()
    t_inject = io.inject()
    stage.chain(t_inject)
    moved = [e for e in io.events if e['kind'] == 'state' and e['t'] > t0 and e['v'] > STILL]
    stage.check('still_throughout', 'robot still throughout', not moved)
    stage.finish_release()
    stage.confounded(t0)


def run_motion(stage: Stage) -> None:
    io, a = stage.io, stage.a
    if not stage.preflight(('armed',)):
        return
    stage.check('standing', 'robot standing (body_height > 0.25 m)', io.body_height > 0.25,
                body_height=io.body_height)
    if io.body_height <= 0.25:
        return
    t0 = time.monotonic()
    stage.stream_vx = a.vx
    moving = io.wait_for(lambda: io.speed > MOVING, 3.0, vx=a.vx, abort_speed=a.abort_speed)
    if not stage.check('moving', f'robot measured moving (> {MOVING} m/s) under vx={a.vx}',
                       moving, v=io.speed):
        stage.stream_vx = None
        io.zero()
        io.spin(2.0)
        return
    # let the gait settle so the measured stop starts from steady motion
    if io.spin(a.settle_s, vx=a.vx, abort_speed=a.abort_speed):
        stage.check('no_overspeed', 'no overspeed', False)
        return
    v_at = io.speed
    t_inject = io.inject()
    stage.check('moving_at_fault', 'moving at injection', v_at > MOVING, v=v_at)
    st = stage.chain(t_inject, held_deadline=3.0)
    # keep streaming upstream for the rest of the hold window
    rest = max(0.0, a.hold_stream_s - (time.monotonic() - t_inject))
    aborted = io.spin(rest, vx=a.vx, abort_speed=a.abort_speed)
    stage.check('no_overspeed', f'overspeed guard never tripped (abort {a.abort_speed} m/s)',
                not aborted)
    held = [e for e in io.events if e['t'] > t_inject]
    first_still = None
    for e in held:
        if e['kind'] == 'state' and e['v'] <= STILL and abs(e['yaw']) <= STILL_YAW:
            if first_still is None:
                first_still = e['t']
        elif e['kind'] == 'state':
            first_still = None
    odom_after = [e['v'] for e in held if e['kind'] == 'odom'
                  and e['t'] > t_inject + a.stop_deadline_s]
    moves_held = [e for e in held if e['kind'] == 'req' and e['api_id'] == 1008
                  and st['hold_requested_ms'] is not None
                  and e['t'] > t_inject + st['hold_requested_ms'] / 1e3 + 0.001]
    stop_ms = (first_still - t_inject) * 1e3 if first_still else None
    stage.check('stopped_measured', f'SportModeState still within {a.stop_deadline_s} s of fault',
                stop_ms is not None and stop_ms <= a.stop_deadline_s * 1e3, stop_ms=stop_ms)
    stage.check('odom_still', 'odometry topic still after the deadline (same estimate as SportModeState)',
                bool(odom_after) and max(odom_after) <= 2 * STILL,
                odom_max=max(odom_after, default=None))
    stage.check('no_move_held', 'no Move sent while held (upstream kept streaming)',
                not moves_held, n=len(moves_held))
    stage.finish_release()
    stage.confounded(t0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage', required=True, choices=('ack', 'motion'))
    ap.add_argument('--out', required=True, help='evidence directory')
    ap.add_argument('--nav-topic', default='/helix_hw/nav/cmd_vel')
    ap.add_argument('--vx', type=float, default=0.15)
    ap.add_argument('--settle-s', type=float, default=1.0)
    ap.add_argument('--stop-deadline-s', type=float, default=1.5)
    ap.add_argument('--hold-stream-s', type=float, default=3.0)
    ap.add_argument('--abort-speed', type=float, default=0.35)
    a = ap.parse_args()
    if not 0.0 < a.vx <= 0.25:
        print('vx must be in (0, 0.25] (sink limit)')
        return 2
    rclpy.init()
    io = Io(a.nav_topic)
    stage = Stage(io, a)
    print(f'=== native stop test, stage {a.stage} ===')
    try:
        (run_ack if a.stage == 'ack' else run_motion)(stage)
    finally:
        io.zero()
    verdict = 'PASS' if stage.checks and all(c['pass'] for c in stage.checks) else 'FAIL'
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    sha = None
    try:
        sha = Path(sys.argv[0]).resolve().parents[2].joinpath('GIT_SHA').read_text().strip()
    except OSError:
        pass
    (out / 'evidence.json').write_text(json.dumps({
        'stage': a.stage, 'verdict': verdict, 'args': vars(a), 'git_sha': sha,
        'utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'checks': stage.checks, 'events': io.events,
    }, indent=1))
    print(f'\nSTAGE {a.stage}: {verdict}\nevidence: {out / "evidence.json"}')
    io.node.destroy_node()
    rclpy.shutdown()
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
