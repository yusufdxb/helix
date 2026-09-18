"""Timestamped command tracing and stage-latency analysis.

Recorder: one process subscribes to every stage of the recovery chain and
writes one JSON line per message, stamped with ITS OWN monotonic receipt
clock. All stages therefore share a single clock, which is the only honest
way to difference them (the robot and payload clocks are skewed).

  fault   /helix/faults            helix_msgs/FaultEvent
  hint    /helix/recovery_hints    helix_msgs/RecoveryHint
  action  /helix/recovery_actions  helix_msgs/RecoveryAction
  hold    /helix/hold              helix_msgs/HelixHold
  status  /helix/arbiter/status    helix_msgs/ArbiterStatus
  output  <output_topic>           geometry_msgs/Twist   (arbiter output)
  sink    /helix/sink/trace        std_msgs/String JSON  (GO2 sport sink)

analyze() correlates by fault_id (== FaultEvent.node_name) and reports, for
every ACCEPTED STOP_AND_HOLD, each stage in two clocks:

  receipt (``times``/``deltas_ms``): the observer's callback time. It proves
      ORDER, but a single-threaded observer drains queued messages in batches,
      so its deltas measure callback order, not transit. Not a latency.
  source (``src_times``/``src_deltas_ms``): the wall-clock stamp each stage's
      publisher wrote into its message. HELIX, the arbiter and the sink run on
      one host (the payload Jetson on the robot, one PC off-robot), so these
      share a clock and their differences ARE the pipeline latency. This is
      the headline number. RecoveryHint carries no stamp, so hint has none.

Source stamps: fault=FaultEvent.timestamp (detector publish), action=
RecoveryAction.timestamp (envelope decision), hold=HelixHold.stamp,
status=ArbiterStatus.stamp (written immediately after the output publish),
sink=trace t_wall (sink decision).
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from typing import Dict, List, Optional

STAGES = ('fault', 'hint', 'action', 'hold', 'status', 'output', 'sink')


def _first(events, pred, after: float) -> Optional[dict]:
    for e in events:
        if e['t'] >= after and pred(e):
            return e
    return None


def analyze(events: List[dict]) -> Dict[str, object]:
    """Build one chain per accepted STOP. Pure; events sorted by receipt t."""
    ev = sorted(events, key=lambda e: e['t'])
    chains = []
    for act in ev:
        if not (act['kind'] == 'action' and act['action'] == 'STOP_AND_HOLD'
                and act['status'] == 'ACCEPTED'):
            continue
        fid = act['fault_id']
        hint = None
        for e in ev:
            if e['t'] > act['t']:
                break
            if (e['kind'] == 'hint' and e['fault_id'] == fid
                    and e['suggested_action'] == 'STOP_AND_HOLD'):
                hint = e
        fault = None
        if hint is not None:
            for e in ev:
                if e['t'] > hint['t']:
                    break
                if e['kind'] == 'fault' and e['node_name'] == fid:
                    fault = e
        t0 = hint['t'] if hint else act['t']
        hold = _first(ev, lambda e: e['kind'] == 'hold' and e['hold']
                      and e['fault_id'] == fid, t0)
        th = hold['t'] if hold else act['t']
        status = _first(ev, lambda e: e['kind'] == 'status'
                        and e['reason'] == 'HELIX_HOLD'
                        and e['hold_fault_id'] == fid, th)
        output = None
        if status is not None:
            output = _first(ev, lambda e: e['kind'] == 'output' and e['zero'], th)
        sink = None
        if output is not None:
            sink = _first(ev, lambda e: e['kind'] == 'sink' and e['api_id'] == 1003,
                          th)
        stages = dict(fault=fault, hint=hint, action=act, hold=hold,
                      status=status, output=output, sink=sink)
        times = {k: (v['t'] if v else None) for k, v in stages.items()}
        src = {k: (v.get('src_stamp') if v else None) for k, v in stages.items()}
        src_deltas = {}
        prev = None
        for k in STAGES:
            if src[k] is None:
                continue
            if prev is not None:
                src_deltas[f'{prev}->{k}_ms'] = round((src[k] - src[prev]) * 1e3, 3)
            prev = k
        deltas = {}
        prev = None
        for k in STAGES:
            if times[k] is None:
                continue
            if prev is not None:
                deltas[f'{prev}->{k}_ms'] = round((times[k] - times[prev]) * 1e3, 3)
            prev = k
        first = next((times[k] for k in STAGES if times[k] is not None), None)
        end = times['output']
        def _src_ms(a, b):
            if src[a] is None or src[b] is None:
                return None
            return round((src[b] - src[a]) * 1e3, 3)
        chains.append({
            'fault_id': fid, 'times': times, 'deltas_ms': deltas,
            'src_times': src, 'src_deltas_ms': src_deltas,
            'fault_to_output_src_ms': _src_ms('fault', 'status'),
            'fault_to_sink_src_ms': _src_ms('fault', 'sink'),
            'action_to_output_src_ms': _src_ms('action', 'status'),
            'complete_to_output': all(times[k] is not None for k in
                                      ('hint', 'action', 'hold', 'status', 'output')),
            'fault_to_output_ms': (round((end - times['fault']) * 1e3, 3)
                                   if end is not None and times['fault'] is not None
                                   else None),
            'first_to_output_ms': (round((end - first) * 1e3, 3)
                                   if end is not None and first is not None else None),
        })
    summary = {}
    for key in ('fault_to_output_src_ms', 'fault_to_sink_src_ms',
                'action_to_output_src_ms', 'fault_to_output_ms'):
        vals = [c[key] for c in chains if c[key] is not None]
        if vals:
            summary[key] = {'n': len(vals), 'median': statistics.median(vals),
                            'max': max(vals), 'min': min(vals)}
    nonzero_after_hold = []
    forced = ('HELIX_HOLD', 'HELIX_STATE_STALE', 'HELIX_STATE_MISSING', 'SHUTDOWN')
    for c in chains:
        th = c['times']['status']
        if th is None:
            continue
        # The hold episode runs from the first HELIX_HOLD status to the LAST
        # forced status before the first unforced one. The window must end at
        # that last forced status, not at the next unforced status: the
        # arbiter publishes each tick's output BEFORE that tick's status, so
        # the first post-release output (one tick, ~20 ms, after the last
        # forced tick) reaches an observer just before its own SOURCE status.
        tend = th
        for e in ev:
            if e['t'] < th or e['kind'] != 'status':
                continue
            if e['reason'] not in forced:
                break
            tend = e['t']
        nonzero_after_hold += [e for e in ev if e['kind'] == 'output'
                               and th <= e['t'] <= tend and not e['zero']]
    return {'chains': chains, 'summary': summary,
            'nonzero_outputs_during_hold': len(nonzero_after_hold)}


def _record(out_path: str, duration: float, output_topic: str) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from helix_msgs.msg import (
        ArbiterStatus,
        FaultEvent,
        HelixHold,
        RecoveryAction,
        RecoveryHint,
    )

    rclpy.init()
    node = rclpy.create_node('helix_trace_recorder')
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    fp = open(out_path, 'w', encoding='utf-8')

    def w(kind, **kw):
        kw.update(kind=kind, t=time.monotonic(), wall=time.time())
        fp.write(json.dumps(kw) + '\n')

    node.create_subscription(FaultEvent, '/helix/faults', lambda m: w(
        'fault', node_name=m.node_name, fault_type=m.fault_type,
        severity=int(m.severity), src_stamp=m.timestamp, detail=m.detail), qos)
    node.create_subscription(RecoveryHint, '/helix/recovery_hints', lambda m: w(
        'hint', fault_id=m.fault_id, suggested_action=m.suggested_action,
        rule=m.rule_matched), qos)
    node.create_subscription(RecoveryAction, '/helix/recovery_actions', lambda m: w(
        'action', fault_id=m.fault_id, action=m.action, status=m.status,
        src_stamp=m.timestamp, reason=m.reason), qos)
    node.create_subscription(HelixHold, '/helix/hold', lambda m: w(
        'hold', hold=bool(m.hold), fault_id=m.fault_id, reason=m.reason,
        epoch=int(m.epoch), seq=int(m.seq), src_stamp=m.stamp), qos)
    node.create_subscription(ArbiterStatus, '/helix/arbiter/status', lambda m: w(
        'status', reason=m.reason, source=m.selected_source,
        hold_fault_id=m.hold_fault_id, vx=m.out_linear_x, vy=m.out_linear_y,
        wz=m.out_angular_z, seq=int(m.seq), rejected=int(m.rejected_total),
        src_stamp=m.stamp,
        sink_subscribers=int(m.sink_subscribers)), qos)
    node.create_subscription(Twist, output_topic, lambda m: w(
        'output', vx=m.linear.x, vy=m.linear.y, wz=m.angular.z,
        zero=(m.linear.x == 0.0 and m.linear.y == 0.0 and m.angular.z == 0.0)), qos)

    def on_sink(m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        w('sink', api_id=d.get('api_id'), reason=d.get('reason'),
          src_stamp=d.get('t_wall'),
          sent_to_robot=d.get('sent_to_robot'), mode=d.get('mode'))
    node.create_subscription(String, '/helix/sink/trace', on_sink, qos)

    end = time.monotonic() + duration if duration > 0 else float('inf')
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        fp.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('--out', required=True)
    r.add_argument('--duration', type=float, default=0.0, help='0 = until Ctrl-C')
    r.add_argument('--output-topic', default='/cmd_vel')
    a = sub.add_parser('analyze')
    a.add_argument('trace')
    a.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)
    if args.cmd == 'record':
        return _record(args.out, args.duration, args.output_topic)
    with open(args.trace, encoding='utf-8') as fp:
        events = [json.loads(line) for line in fp if line.strip()]
    res = analyze(events)
    if args.json:
        print(json.dumps(res, indent=1))
    else:
        for c in res['chains']:
            print(f"{c['fault_id']}: complete={c['complete_to_output']} "
                  f"src_deltas={c['src_deltas_ms']} "
                  f"fault->output={c['fault_to_output_src_ms']} ms (source clock)")
        print('summary', json.dumps(res['summary']))
        print('nonzero outputs during hold:', res['nonzero_outputs_during_hold'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
