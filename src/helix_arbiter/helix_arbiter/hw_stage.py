"""Bounded GO2 hardware test runner for the HELIX motion path, stages A-F.

ONE invocation runs ONE stage and exits. Nothing ever proceeds to a more
dangerous stage automatically:

  * stage X refuses to run unless <session-dir>/stage_<X-1>/evidence.json is
    PASS with the SAME git SHA and the SAME config hash;
  * the stage's preflight must be GO;
  * the operator must type the stage's confirmation phrase (or pass it with
    --confirm, used only by the off-robot rehearsal).

The runner never commands the robot directly. Its only outputs are
/nav/cmd_vel (an ordinary upstream source that must go THROUGH the arbiter)
and, in C/E/F, one benign synthetic FaultEvent on /helix/faults (it does not
touch any sensor). The GO2 sport sink's mode (dry_run / stop_only / armed) is
set by the operator when launching it, and preflight check C7 verifies it.

Dry variants: the dry_run stages (A, C) can also run with the motion topics
remapped onto sink topics (--topic-prefix, or --cmd-topic / --nav-topic /
--teleop-topic), with the arbiter and sink launched on the same topics. Such
evidence is marked `topics: remapped`, lives in its own session dir (marker
file TOPICS_REMAPPED), chains A -> C, and never unlocks a real-topic stage.

  A  graph + topic verification, motors off           sink dry_run
  B  live arbitration, zero velocity only             sink stop_only
  C  low-speed command visible through the arbiter,   sink dry_run
     robot physically staged; STOP chain exercised
  D  one short bounded low-speed movement             sink armed
  E  benign fault injected while moving: STOP_AND_HOLD, sink armed
     arbiter zero, robot physically stops
  F  release / re-arm under operator control,         sink armed
     no spontaneous motion

Each stage writes <session-dir>/stage_<X>/{evidence.json, trace.jsonl,
preflight.json} with git SHA, config hash, ROS graph, input commands, HELIX
faults, RecoveryHints, RecoveryActions, holds, arbiter selections, sink
decisions, robot odometry, timestamps, and PASS / FAIL / INCOMPLETE.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import socket
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

from helix_arbiter import preflight
from helix_arbiter.preflight import DEFAULT_TOPICS, REMAP_STAGES

STAGES = 'ABCDEF'
REMAPPED_MARKER = 'TOPICS_REMAPPED'
# Matches diagnosis rule R1 (prefix rate_hz/utlidar) but can never be a real
# metric name, so evidence cannot confuse the injected fault with a real one.
INJECTED_FAULT_ID = 'rate_hz/utlidar_helix_injected'
PASS, FAIL, INCOMPLETE = 'PASS', 'FAIL', 'INCOMPLETE'

# Motion envelope (m/s). Well inside the measured GO2 envelope: commanded
# 0.15 m/s produced a 0.159-0.161 m/s peak (GO2_FIELD_NOTES.md section 4).
TEST_VX = 0.15
STAGED_VX = 0.10
ABORT_SPEED = 0.35          # runner stops sourcing commands above this
STOPPED_SPEED = 0.03        # "physically stopped"
STOP_DEADLINE_S = 1.5       # after the hold is asserted
MOVE_S = 2.0

CONFIRM = {
    'A': 'MOTORS OFF',
    'B': 'ZERO ONLY',
    'C': 'ROBOT STAGED',
    'D': 'AREA CLEAR MOVE',
    'E': 'AREA CLEAR FAULT',
    'F': 'OPERATOR REARM',
}
CHECKLIST = {
    'A': ['Robot powered, motors OFF / damped, lying down', 'Sink launched mode:=dry_run',
          'E-stop / remote in operator hand'],
    'B': ['Robot standing in mcf mode, operator holds remote', 'Sink relaunched mode:=stop_only',
          'Nothing but StopMove can reach the robot'],
    'C': ['Robot physically staged (sitting / held / feet off ground)',
          'Sink relaunched mode:=dry_run (nothing reaches the robot)'],
    'D': ['Robot standing, 2 m clear ahead, spotter in place, remote in hand',
          'Sink relaunched mode:=armed', f'Command {TEST_VX} m/s for {MOVE_S} s max'],
    'E': ['As D. A synthetic ANOMALY FaultEvent will be injected mid-motion',
          'Upstream command KEEPS streaming; only the HELIX hold may stop the robot'],
    'F': ['As D. No upstream command will be sent', 'Operator performs the re-arm'],
}


# --------------------------------------------------------------------------
# pure helpers (unit-tested)
# --------------------------------------------------------------------------

def topics_mode(ev: dict) -> str:
    """Topic mode of stage evidence. Evidence written before the remap option
    existed carries no 'topics' key and was always on the real topics."""
    return (ev.get('topics') or {}).get('mode', 'real')


def remap_refusal(stage: str, topics: dict) -> Optional[str]:
    """Remapped topics are allowed only for the dry_run stages."""
    if topics['mode'] == 'remapped' and stage not in REMAP_STAGES:
        return (f'stage {stage} cannot run on remapped topics: only the dry_run '
                f'stages {"/".join(REMAP_STAGES)} have a dry variant')
    return None


def session_refusal(session: Path, remapped: bool) -> Optional[str]:
    """Keep real and remapped evidence in separate session dirs."""
    marked = (session / REMAPPED_MARKER).exists()
    if marked and not remapped:
        return ('session dir is a TOPICS REMAPPED (dry) session; use a fresh dir '
                'for real-topic stages')
    if remapped and not marked and any(session.glob('stage_*/evidence.json')):
        return 'session dir holds real-topic evidence; use a fresh dir for dry stages'
    return None


def gate(stage: str, session: Path, sha: str, cfg_hash: str,
         mode: str = 'real') -> Optional[str]:
    """Return a refusal reason, or None if the stage may run.

    ``mode`` is the topic mode of the stage about to run. Remapped (dry)
    stages chain over the dry_run stages only (A -> C); a real stage accepts
    only real-topic evidence, so a sink-topic PASS can never unlock it.
    """
    order = ''.join(REMAP_STAGES) if mode == 'remapped' else STAGES
    i = order.index(stage)
    if i == 0:
        return None
    prev = order[i - 1]
    p = session / f'stage_{prev}' / 'evidence.json'
    if not p.exists():
        return f'stage {prev} evidence missing ({p}); run stages in order'
    ev = json.loads(p.read_text())
    if ev.get('verdict') != PASS:
        return f'stage {prev} verdict is {ev.get("verdict")}, not PASS'
    if topics_mode(ev) != mode:
        return (f'stage {prev} evidence has topics: {topics_mode(ev)}, this run is {mode}; '
                'remapped (sink-topic) evidence never unlocks a real-topic stage')
    if ev.get('git_sha') != sha:
        return f'git SHA changed since stage {prev}: {ev.get("git_sha")} -> {sha}'
    if ev.get('config_hash') != cfg_hash:
        return f'config hash changed since stage {prev}'
    if ev.get('rehearsal') != (session / 'REHEARSAL').exists():
        return 'rehearsal/hardware mismatch within one session dir'
    return None


def speeds_from_odom(odom: List[dict]) -> List[dict]:
    """Add a pose-differenced speed (>=50 ms baseline) to each odom sample."""
    out, j = [], 0
    for i, o in enumerate(odom):
        while j < i and o['t'] - odom[j]['t'] > 0.05:
            j += 1
        k = max(0, j - 1)
        dt = o['t'] - odom[k]['t']
        v = (math.hypot(o['x'] - odom[k]['x'], o['y'] - odom[k]['y']) / dt) if dt > 0 else 0.0
        out.append({**o, 'speed_pose': v, 'speed_twist': math.hypot(o['vx'], o['vy'])})
    return out


def speed(o: dict) -> float:
    return max(o['speed_pose'], o['speed_twist'])


def stop_metrics(odom: List[dict], t_hold: float) -> Dict[str, Optional[float]]:
    """Time and distance from hold assertion until speed stays below STOPPED_SPEED."""
    after = [o for o in odom if o['t'] >= t_hold]
    if not after:
        return {'stop_time_s': None, 'stop_distance_m': None, 'speed_at_hold': None}
    stopped_at = None
    for i, o in enumerate(after):
        if all(speed(p) < STOPPED_SPEED for p in after[i:]):
            stopped_at = o
            break
    base = after[0]
    dist = None
    if stopped_at is not None:
        dist = math.hypot(stopped_at['x'] - base['x'], stopped_at['y'] - base['y'])
    return {'stop_time_s': None if stopped_at is None else stopped_at['t'] - t_hold,
            'stop_distance_m': dist, 'speed_at_hold': speed(base)}


def config_hash(files: Dict[str, str], params: Dict[str, dict]) -> str:
    h = hashlib.sha256()
    for k in sorted(files):
        h.update(k.encode())
        h.update(files[k].encode())
    h.update(json.dumps(params, sort_keys=True).encode())
    return h.hexdigest()


# --------------------------------------------------------------------------
# ROS I/O
# --------------------------------------------------------------------------

class StageIO:
    """Recorder + the runner's only two publishers (nav source, /helix/faults).

    ``topics`` is a preflight.resolve_topics() result; the arbiter output
    ('cmd') and nav source ('nav') come from it, defaulting to the real path.
    """

    def __init__(self, rehearsal: bool, topics: Optional[dict] = None):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from std_msgs.msg import String

        from helix_msgs.msg import (
            ArbiterStatus,
            FaultEvent,
            HelixHold,
            RecoveryAction,
            RecoveryHint,
        )
        self.rclpy = rclpy
        self.topics = topics or {'mode': 'real', **DEFAULT_TOPICS}
        cmd_t, nav_t = self.topics['cmd'], self.topics['nav']
        self.Twist, self.FaultEvent = Twist, FaultEvent
        self.node = rclpy.create_node('helix_hw_stage')
        n = self.node
        q = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        self.events: List[dict] = []
        self.odom: List[dict] = []
        self.nav = n.create_publisher(Twist, nav_t, 10)
        self.faults = n.create_publisher(FaultEvent, '/helix/faults', 10)
        ev = self._ev
        n.create_subscription(FaultEvent, '/helix/faults', lambda m: ev(
            'fault', node_name=m.node_name, fault_type=m.fault_type,
            severity=int(m.severity), detail=m.detail, src_stamp=m.timestamp), q)
        n.create_subscription(RecoveryHint, '/helix/recovery_hints', lambda m: ev(
            'hint', fault_id=m.fault_id, suggested_action=m.suggested_action,
            rule=m.rule_matched, reasoning=m.reasoning), q)
        n.create_subscription(RecoveryAction, '/helix/recovery_actions', lambda m: ev(
            'action', fault_id=m.fault_id, action=m.action, status=m.status,
            reason=m.reason, src_stamp=m.timestamp), q)
        n.create_subscription(HelixHold, '/helix/hold', lambda m: ev(
            'hold', hold=bool(m.hold), fault_id=m.fault_id, reason=m.reason,
            seq=int(m.seq), epoch=int(m.epoch), src_stamp=m.stamp), q)
        n.create_subscription(ArbiterStatus, '/helix/arbiter/status', lambda m: ev(
            'status', reason=m.reason, source=m.selected_source,
            hold_fault_id=m.hold_fault_id, vx=m.out_linear_x, vy=m.out_linear_y,
            wz=m.out_angular_z, rejected=int(m.rejected_total),
            sink_subscribers=int(m.sink_subscribers), src_stamp=m.stamp), q)
        n.create_subscription(Twist, cmd_t, lambda m: ev(
            'output', vx=m.linear.x, vy=m.linear.y, wz=m.angular.z,
            zero=(m.linear.x == 0.0 and m.linear.y == 0.0 and m.angular.z == 0.0)), q)
        n.create_subscription(Twist, nav_t, lambda m: ev(
            'input', vx=m.linear.x, wz=m.angular.z), q)

        def on_sink(m):
            d = json.loads(m.data)
            ev('sink', api_id=d['api_id'], reason=d['reason'], x=d['x'], y=d['y'],
               z=d['z'], sent_to_robot=d['sent_to_robot'], mode=d['mode'],
               request_id=d['request_id'], src_stamp=d['t_wall'])
        n.create_subscription(String, '/helix/sink/trace', on_sink, q)

        def on_odom(m):
            p, t = m.pose.pose.position, m.twist.twist
            self.odom.append({'t': time.monotonic(), 'x': p.x, 'y': p.y,
                              'vx': t.linear.x, 'vy': t.linear.y, 'wz': t.angular.z,
                              'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9})
        n.create_subscription(Odometry, '/utlidar/robot_odom', on_odom, qos_profile_sensor_data)
        self.responses_available = False
        try:
            from unitree_api.msg import Response
            n.create_subscription(Response, '/api/sport/response', lambda m: ev(
                'response', request_id=int(m.header.identity.id),
                api_id=int(m.header.identity.api_id), code=int(m.header.status.code)), q)
            self.responses_available = True
        except ImportError:
            pass

    def _ev(self, kind, **kw):
        kw.update(kind=kind, t=time.monotonic(), wall=time.time())
        self.events.append(kw)

    def spin(self, duration: float, vx: Optional[float] = None,
             each: Optional[Callable[[], Optional[str]]] = None) -> Optional[str]:
        """Spin; stream nav vx at 20 Hz if given. Returns an abort reason or None."""
        end = time.monotonic() + duration
        nxt = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= nxt:
                if vx is not None:
                    t = self.Twist()
                    t.linear.x = float(vx)
                    self.nav.publish(t)
                nxt = now + 0.05
                if each:
                    why = each()
                    if why:
                        return why
                why = self.overspeed()
                if why:
                    self.zero_and_release()
                    return why
            self.rclpy.spin_once(self.node, timeout_sec=0.005)
        return None

    def overspeed(self) -> Optional[str]:
        o = speeds_from_odom(self.odom[-20:])
        if o and speed(o[-1]) > ABORT_SPEED:
            return f'ABORT_OVERSPEED {speed(o[-1]):.3f} m/s > {ABORT_SPEED}'
        return None

    def zero_and_release(self) -> None:
        for _ in range(5):
            self.nav.publish(self.Twist())
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def inject_fault(self) -> float:
        f = self.FaultEvent()
        f.node_name = INJECTED_FAULT_ID
        f.fault_type = 'ANOMALY'
        f.severity = 2
        f.detail = 'INJECTED by helix_hw_stage: synthetic benign rate anomaly (no sensor touched)'
        f.timestamp = time.time()
        f.context_keys = ['metric_name', 'injected']
        f.context_values = [INJECTED_FAULT_ID, 'true']
        t = time.monotonic()
        self.faults.publish(f)
        return t

    def lifecycle(self, node_name: str, transition_id: int) -> bool:
        from lifecycle_msgs.srv import ChangeState
        cli = self.node.create_client(ChangeState, f'/{node_name}/change_state')
        try:
            if not cli.wait_for_service(timeout_sec=5.0):
                return False
            req = ChangeState.Request()
            req.transition.id = transition_id
            fut = cli.call_async(req)
            end = time.monotonic() + 10
            while not fut.done() and time.monotonic() < end:
                self.rclpy.spin_once(self.node, timeout_sec=0.02)
            return bool(fut.done() and fut.result().success)
        finally:
            self.node.destroy_client(cli)

    def node_params(self, node_name: str, exclude=()) -> Dict[str, object]:
        from rcl_interfaces.srv import GetParameters, ListParameters
        out: Dict[str, object] = {}
        lc = self.node.create_client(ListParameters, f'{node_name}/list_parameters')
        gc = self.node.create_client(GetParameters, f'{node_name}/get_parameters')
        try:
            if not (lc.wait_for_service(timeout_sec=3.0) and gc.wait_for_service(timeout_sec=3.0)):
                return {'_error': 'unreachable'}
            fut = lc.call_async(ListParameters.Request())
            self._wait(fut)
            names = [nm for nm in fut.result().result.names
                     if nm not in exclude and nm != 'use_sim_time']
            fut = gc.call_async(GetParameters.Request(names=names))
            self._wait(fut)
            for nm, v in zip(names, fut.result().values):
                out[nm] = _param_value(v)
        finally:
            self.node.destroy_client(lc)
            self.node.destroy_client(gc)
        return out

    def _wait(self, fut, timeout=5.0):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    # -- queries --
    def of(self, kind: str, since: float = 0.0, until: float = math.inf) -> List[dict]:
        return [e for e in self.events if e['kind'] == kind and since <= e['t'] < until]


def _param_value(v):
    t = v.type
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value,
            6: list(v.bool_array_value), 7: list(v.integer_array_value),
            8: list(v.double_array_value), 9: list(v.string_array_value)}.get(t)


# --------------------------------------------------------------------------
# stages
# --------------------------------------------------------------------------

class Stage:
    def __init__(self, io: StageIO, rehearsal: bool):
        self.io, self.rehearsal = io, rehearsal
        self.checks: List[dict] = []
        self.notes: Dict[str, object] = {}
        self.abort: Optional[str] = None

    def check(self, cid, name, ok, detail='', required=True):
        self.checks.append({'id': cid, 'name': name, 'result': PASS if ok else FAIL,
                            'required': required, 'detail': str(detail)})
        return ok

    def robot_still(self, since, until=math.inf, cid='still', name='robot stationary'):
        od = [o for o in speeds_from_odom(self.io.odom) if since <= o['t'] < until]
        peak = max((speed(o) for o in od), default=None)
        return self.check(cid, name, peak is not None and peak < STOPPED_SPEED,
                          f'peak speed {peak} m/s over {len(od)} odom samples')

    def no_nonzero_output(self, since, until=math.inf, cid='zero_out',
                          name='arbiter output all zero'):
        outs = self.io.of('output', since, until)
        bad = [o for o in outs if not o['zero']]
        return self.check(cid, name, outs and not bad,
                          f'{len(outs)} outputs, {len(bad)} nonzero')

    def no_move_sent(self, since, until=math.inf, cid='no_move', name='no Move sent to robot'):
        mv = [e for e in self.io.of('sink', since, until)
              if e['api_id'] == 1008 and e['sent_to_robot']]
        return self.check(cid, name, not mv, f'{len(mv)} Move requests sent')

    def wait_resume(self, since, timeout=8.0):
        def done():
            return 'resumed' if any(e['suggested_action'] == 'RESUME'
                                    for e in self.io.of('hint', since)) else None
        return self.io.spin(timeout, each=done) == 'resumed'

    def hold_chain(self, t_fault):
        from helix_arbiter.trace import analyze
        res = analyze([e for e in self.io.events if e['t'] >= t_fault - 0.01])
        chains = [c for c in res['chains'] if c['fault_id'] == INJECTED_FAULT_ID]
        self.notes['chain'] = chains[0] if chains else None
        self.notes['nonzero_outputs_during_hold'] = res['nonzero_outputs_during_hold']
        c = chains[0] if chains else None
        self.check('chain', 'fault->hint->action->hold->arbiter zero->output zero',
                   bool(c and c['complete_to_output']),
                   c and {k: v for k, v in c['times'].items()})
        self.check('hold_zero', 'no nonzero output while held',
                   res['nonzero_outputs_during_hold'] == 0,
                   res['nonzero_outputs_during_hold'])
        return c

    # ---- A ----
    def run_A(self):
        t0 = time.monotonic()
        self.abort = self.io.spin(5.0)
        self.no_nonzero_output(t0)
        self.no_move_sent(t0)
        self.robot_still(t0)

    # ---- B ----
    def run_B(self):
        t0 = time.monotonic()
        self.abort = self.io.spin(5.0, vx=0.0)
        st = self.io.of('status', t0 + 0.5)
        self.check('selected', 'arbiter selected nav (zero)',
                   st and all(s['reason'] == 'SOURCE' and s['source'] == 'nav'
                              and s['vx'] == 0.0 for s in st[-20:]),
                   st[-1] if st else None)
        self.no_nonzero_output(t0)
        stops = [e for e in self.io.of('sink', t0) if e['api_id'] == 1003 and e['sent_to_robot']]
        self.check('stopmove_sent', 'StopMove reached the robot', len(stops) >= 1, len(stops))
        self.no_move_sent(t0)
        if self.io.responses_available:
            ids = {e['request_id'] for e in stops}
            acc = [r for r in self.io.of('response', t0) if r['request_id'] in ids
                   and r['code'] == 0]
            self.check('robot_accepts', 'robot answered StopMove with code 0', acc, len(acc))
        else:
            self.check('robot_accepts', 'robot answered StopMove with code 0', False,
                       'unitree_api not importable: cannot observe /api/sport/response')
        self.robot_still(t0)

    # ---- C ----
    def run_C(self):
        t0 = time.monotonic()
        self.abort = self.io.spin(1.5, vx=STAGED_VX)
        outs = self.io.of('output', t0 + 0.5)
        self.check('visible', f'{STAGED_VX} m/s visible through the arbiter',
                   outs and all(abs(o['vx'] - STAGED_VX) < 1e-9 for o in outs),
                   f'{len(outs)} outputs')
        mv = [e for e in self.io.of('sink', t0) if e['api_id'] == 1008]
        self.check('sink_would_move', 'sink decided Move (dry_run, not sent)',
                   mv and not any(e['sent_to_robot'] for e in mv), f'{len(mv)} Move decisions')
        t_f = self.io.inject_fault()
        self.abort = self.abort or self.io.spin(1.5, vx=STAGED_VX)
        c = self.hold_chain(t_f)
        if c and c['times']['hold']:
            stops = [e for e in self.io.of('sink', c['times']['hold']) if e['api_id'] == 1003]
            self.check('sink_would_stop', 'sink decided StopMove(ZERO) after hold',
                       stops and stops[0]['reason'] == 'ZERO', stops[:1])
        self.io.spin(0.2)
        self.wait_resume(t_f)
        self.robot_still(t0)
        self.no_move_sent(t0)

    # ---- D ----
    def run_D(self):
        t0 = time.monotonic()
        self.abort = self.io.spin(MOVE_S, vx=TEST_VX)
        t_zero = time.monotonic()
        self.abort = self.abort or self.io.spin(1.0, vx=0.0)
        self.io.spin(1.0)
        od = speeds_from_odom(self.io.odom)
        peak = max((speed(o) for o in od if o['t'] >= t0), default=0.0)
        self.notes['peak_speed'] = peak
        self.notes.update(stop_metrics(od, t_zero))
        self.check('moved', f'robot moved (peak in [0.05, 0.25] for {TEST_VX} cmd)',
                   0.05 <= peak <= 0.25, f'peak {peak:.3f} m/s')
        sent = [e for e in self.io.of('sink', t_zero) if e['api_id'] == 1003 and e['sent_to_robot']]
        self.check('stopmove_sent', 'StopMove sent after command went to zero', sent, len(sent))
        st = self.notes.get('stop_time_s')
        self.check('stopped', f'robot stopped within {STOP_DEADLINE_S} s of zero command',
                   st is not None and st <= STOP_DEADLINE_S, self.notes)

    # ---- E ----
    def run_E(self):
        t0 = time.monotonic()
        state = {'t_f': None}

        def maybe_inject():
            od = speeds_from_odom(self.io.odom[-20:])
            # inject at near-steady speed so the measured stop is meaningful
            moving = od and speed(od[-1]) > 0.8 * TEST_VX
            if state['t_f'] is None and (moving or time.monotonic() - t0 > 1.5):
                state['t_f'] = self.io.inject_fault()
            return None
        # Upstream keeps streaming TEST_VX for the whole 3 s: only the hold may stop it.
        self.abort = self.io.spin(3.0, vx=TEST_VX, each=maybe_inject)
        t_f = state['t_f']
        if t_f is None:
            self.check('injected', 'fault injected', False)
            return
        od0 = speeds_from_odom(self.io.odom)
        self.notes['speed_before_fault'] = max(
            (speed(o) for o in od0 if t_f - 0.3 <= o['t'] < t_f), default=None)
        self.check('moving_at_fault', 'robot moving (>0.05 m/s) when fault injected',
                   (self.notes['speed_before_fault'] or 0) > 0.05,
                   self.notes['speed_before_fault'])
        c = self.hold_chain(t_f)
        self.io.spin(1.0)
        if c and c['times']['hold']:
            th = c['times']['hold']
            sent = [e for e in self.io.of('sink', th) if e['api_id'] == 1003 and e['sent_to_robot']]
            self.check('stopmove_sent', 'StopMove reached the robot after hold', sent,
                       sent[:1])
            self.no_move_sent(th + 0.05, cid='no_move_held',
                              name='no Move sent while held (upstream still streaming)')
            if self.io.responses_available and sent:
                acc = [r for r in self.io.of('response', th)
                       if r['request_id'] == sent[0]['request_id'] and r['code'] == 0]
                self.check('robot_accepts', 'robot acknowledged the StopMove', acc)
            m = stop_metrics(speeds_from_odom(self.io.odom), th)
            self.notes.update(m)
            self.check('physically_stopped',
                       f'robot physically stopped within {STOP_DEADLINE_S} s of hold',
                       m['stop_time_s'] is not None and m['stop_time_s'] <= STOP_DEADLINE_S, m)
        t_r = time.monotonic()
        resumed = self.wait_resume(t_f)
        self.check('resume', 'diagnosis released the hold (RESUME)', resumed)
        t_after = time.monotonic()
        self.io.spin(3.0)
        self.robot_still(t_after, cid='no_spontaneous', name='no motion after RESUME')
        self.no_move_sent(t_r, cid='no_move_after_resume', name='no Move after RESUME')

    # ---- F ----
    def run_F(self):
        from lifecycle_msgs.msg import Transition
        t0 = time.monotonic()
        t_f = self.io.inject_fault()
        self.io.spin(1.0)
        held = any(s['reason'] == 'HELIX_HOLD' for s in self.io.of('status', t_f))
        self.check('held', 'hold asserted', held)
        self.check('resume', 'RESUME released the hold', self.wait_resume(t_f))
        t1 = time.monotonic()
        self.io.spin(5.0)
        self.robot_still(t1, cid='still_after_resume', name='no motion 5 s after RESUME')
        self.no_nonzero_output(t1, cid='zero_after_resume', name='output zero after RESUME')
        print('\nOPERATOR RE-ARM: recovery will be deactivated then reactivated.')
        if not self.rehearsal:
            input('Press Enter to deactivate recovery (robot must stay still)... ')
        ok_d = self.io.lifecycle('helix_recovery_node', Transition.TRANSITION_DEACTIVATE)
        t2 = time.monotonic()
        self.io.spin(2.0)
        forced = [s for s in self.io.of('status', t2 + 0.1)
                  if s['reason'] in ('HELIX_HOLD', 'HELIX_STATE_STALE')]
        self.check('deactivate_holds', 'recovery deactivate forces arbiter zero',
                   ok_d and forced, f'deactivate ok={ok_d}, forced ticks={len(forced)}')
        if not self.rehearsal:
            input('Press Enter to reactivate recovery... ')
        ok_a = self.io.lifecycle('helix_recovery_node', Transition.TRANSITION_ACTIVATE)
        t3 = time.monotonic()
        self.io.spin(5.0)
        self.check('reactivated', 'recovery reactivated', ok_a)
        self.robot_still(t3, cid='still_after_rearm', name='no motion 5 s after re-arm')
        self.no_nonzero_output(t0, cid='zero_all', name='output zero for all of stage F')
        self.no_move_sent(t0)
        st = self.io.of('status', t3 + 1.0)
        self.check('rearmed', 'arbiter back to NO_LIVE_INPUT (released, idle)',
                   st and st[-1]['reason'] == 'NO_LIVE_INPUT', st[-1] if st else None)


# --------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='HELIX GO2 bounded hardware stage runner')
    ap.add_argument('--stage', required=True, choices=list(STAGES))
    ap.add_argument('--session-dir', required=True)
    ap.add_argument('--repo', default=os.environ.get('HELIX_REPO', os.getcwd()),
                    help='helix git checkout (for SHA)')
    ap.add_argument('--rehearsal', action='store_true',
                    help='off-robot run against helix_fake_go2; evidence labelled REHEARSAL')
    ap.add_argument('--confirm', help='confirmation phrase (rehearsal scripting only)')
    preflight.add_topic_args(ap)
    a = ap.parse_args(argv)
    try:
        topics = preflight.topics_from_args(a)
    except ValueError as e:
        ap.error(str(e))
    why = remap_refusal(a.stage, topics)
    if why:
        ap.error(why)
    remapped = topics['mode'] == 'remapped'

    import rclpy
    from ament_index_python.packages import get_package_share_directory

    session = Path(a.session_dir)
    session.mkdir(parents=True, exist_ok=True)
    if a.rehearsal:
        (session / 'REHEARSAL').touch()
    elif (session / 'REHEARSAL').exists():
        print('session dir is a REHEARSAL session; use a fresh dir for hardware')
        return 3
    why = session_refusal(session, remapped)
    if why:
        print(why)
        return 3
    if remapped:
        (session / REMAPPED_MARKER).touch()
    out = session / f'stage_{a.stage}'
    prior = out / 'evidence.json'
    if prior.exists():
        if json.loads(prior.read_text()).get('verdict') == PASS:
            print(f'{out} already PASSED; PASS evidence is never overwritten')
            return 3
        prior.rename(out / f'evidence_failed_{int(time.time())}.json')
    out.mkdir(parents=True, exist_ok=True)
    ran = False

    rclpy.init()
    io = StageIO(a.rehearsal, topics)
    ev: Dict[str, object] = {'stage': a.stage, 'rehearsal': a.rehearsal,
                             'topics': topics,
                             'host': socket.gethostname(),
                             'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
                             'rmw': os.environ.get('RMW_IMPLEMENTATION'),
                             'wall_start': time.time()}
    verdict = INCOMPLETE
    stage = Stage(io, a.rehearsal)
    try:
        gi = preflight.git_info(a.repo)
        files = {}
        for pkg, rel in (('helix_arbiter', 'config/arbiter.yaml'),
                         ('helix_bringup', 'config/helix_params.yaml'),
                         ('helix_bringup', 'launch/helix_closedloop.launch.py')):
            p = Path(get_package_share_directory(pkg)) / rel
            files[f'{pkg}/{rel}'] = p.read_text() if p.exists() else '<missing>'
        io.spin(1.0)
        params = {'arbiter': io.node_params('/helix_arbiter'),
                  'recovery': io.node_params('/helix_recovery_node'),
                  'sink': io.node_params('/helix_go2_sport_sink', exclude=('mode',))}
        ev.update(git_sha=gi['sha'], git_dirty=gi['dirty'],
                  config_hash=config_hash(files, params), params=params)
        if gi['dirty'] and not a.rehearsal:
            ev['refused'] = 'git tree dirty: hardware evidence must map to a commit'
            return 3
        why = gate(a.stage, session, gi['sha'], ev['config_hash'], topics['mode'])
        if why:
            ev['refused'] = why
            return 3
        baseline = session / 'sport_baseline.json'
        if a.stage == 'A':
            pubs = sorted({preflight._qos_dict(e)['node'] for e in
                           io.node.get_publishers_info_by_topic('/api/sport/request')})
            baseline.write_text(json.dumps(pubs, indent=1))
            ev['sport_baseline'] = pubs
        rep = preflight.run(a.stage, str(out / 'preflight.json'),
                            str(baseline) if baseline.exists() else None,
                            a.rehearsal, node=io.node, repo=a.repo, topics=topics)
        preflight.print_report(rep)
        ev['preflight'] = {'verdict': rep['verdict'], 'checks': rep['checks']}
        ev['ros_graph'] = {'nodes': rep['snapshot']['nodes'],
                           'topics': {t: {'types': v['types'],
                                          'publishers': [p['node'] for p in v['publishers']],
                                          'subscribers': [s['node'] for s in v['subscribers']]}
                                      for t, v in rep['snapshot']['topics'].items()}}
        if rep['verdict'] != 'GO':
            ev['refused'] = 'preflight NO-GO'
            return 1
        print(f'\n=== STAGE {a.stage}{" (DRY: TOPICS REMAPPED)" if remapped else ""} ===')
        if remapped:
            print(f"  motion topics remapped: cmd={topics['cmd']} nav={topics['nav']} "
                  f"teleop={topics['teleop']}")
        for item in CHECKLIST[a.stage]:
            print(f'  [ ] {item}')
        phrase = a.confirm if a.confirm is not None else input(
            f'Type "{CONFIRM[a.stage]}" to run stage {a.stage}: ')
        if phrase.strip() != CONFIRM[a.stage]:
            ev['refused'] = 'operator did not confirm'
            print('not confirmed; nothing was run')
            return 3
        ev['confirmed_wall'] = time.time()
        io_t0 = time.monotonic()
        ran = True
        getattr(stage, f'run_{a.stage}')()
        if stage.abort:
            stage.check('abort', 'no abort', False, stage.abort)
        real = [e for e in io.of('action', io_t0)
                if e['action'] == 'STOP_AND_HOLD' and e['status'] == 'ACCEPTED'
                and e['fault_id'] != INJECTED_FAULT_ID]
        stage.check('unconfounded', 'no real (uninjected) HELIX stop during the stage',
                    not real, [r['fault_id'] for r in real])
        req = [c for c in stage.checks if c['required']]
        verdict = PASS if req and all(c['result'] == PASS for c in req) else FAIL
    except KeyboardInterrupt:
        ev['interrupted'] = True
    finally:
        try:
            io.zero_and_release()
        except Exception:  # noqa: BLE001  best-effort on the way out
            pass
        holds = io.of('hold')
        transitions = [h for i, h in enumerate(holds)
                       if i == 0 or h['hold'] != holds[i - 1]['hold']]
        ev.update(verdict=verdict, checks=stage.checks, measurements=stage.notes,
                  wall_end=time.time(), responses_observable=io.responses_available,
                  inputs=[e for e in io.events if e['kind'] == 'input'][:2000:10],
                  helix_faults=io.of('fault'),
                  uninjected_faults=[f for f in io.of('fault')
                                     if f['node_name'] != INJECTED_FAULT_ID],
                  recovery_hints=io.of('hint'),
                  recovery_actions=io.of('action'),
                  hold_transitions=transitions,
                  selected=[s for s in io.of('status')][::10],
                  sink=io.of('sink'))
        if not ran:
            print(f"REFUSED / NOT RUN: {ev.get('refused', 'interrupted before confirmation')}")
            # Refused before any command: keep the record, do not lock the stage.
            att = session / 'attempts'
            att.mkdir(exist_ok=True)
            (att / f'stage_{a.stage}_{int(time.time())}.json').write_text(
                json.dumps(ev, indent=1, default=str))
        else:
            (out / 'evidence.json').write_text(json.dumps(ev, indent=1, default=str))
        with open(out / ('trace.jsonl' if ran else 'refused_trace.jsonl'), 'w') as fp:
            for e in io.events:
                fp.write(json.dumps(e) + '\n')
            for o in io.odom:
                fp.write(json.dumps({'kind': 'odom', **o}) + '\n')
        io.node.destroy_node()
        rclpy.shutdown()
        tag = ' [REHEARSAL: NOT HARDWARE EVIDENCE]' if a.rehearsal else ''
        if remapped:
            tag += ' [TOPICS REMAPPED: NOT REAL COMMAND-PATH EVIDENCE]'
        for c in stage.checks:
            print(f"[{c['result']}] {c['id']}: {c['name']}  {c['detail'][:140]}")
        print(f'\nSTAGE {a.stage}: {verdict}{tag}\nevidence: {out / "evidence.json"}')
    return 0 if verdict == PASS else 1


if __name__ == '__main__':
    sys.exit(main())
