#!/usr/bin/env python3
"""Replay the REAL Session 8 GO2 FaultEvents through the new motion path.

Source: the Session 8 CaresLab bag (post_fix_demo), recorded on the live GO2
+ Jetson on 2026-04-23. Only /helix/faults is replayed; it is real HELIX
sensing output from the robot. Everything downstream runs off-robot, as real
processes: diagnosis -> recovery -> arbiter -> GO2 sport sink (dry_run), with
a fake upstream nav source streaming a constant low-speed command and a fake
final consumer on /cmd_vel.

This is NOT hardware evidence of stopping. It shows how the new path
responds to the fault stream the robot actually produced.

Timing: faults are republished at their original relative times, except that
any idle gap longer than --max-gap seconds is shortened to --max-gap. Every
behaviour-relevant interval in the pipeline (3 s diagnosis clear window, 5 s
recovery cooldown, 0.5 s timeouts) is shorter than the default 10 s cap, so
compression cannot change a decision; the gaps and cap are written to the
result for inspection.

Usage:
  python3 scripts/replay_session8_through_arbiter.py \
      --bag <path to post_fix_demo bag dir> --out results/session8_arbiter_replay.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def read_bag(uri: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    from helix_msgs.msg import FaultEvent, RecoveryAction, RecoveryHint
    types = {'/helix/faults': FaultEvent, '/helix/recovery_hints': RecoveryHint,
             '/helix/recovery_actions': RecoveryAction}
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    out = {k: [] for k in types}
    while r.has_next():
        topic, data, t = r.read_next()
        out[topic].append((t / 1e9, deserialize_message(data, types[topic])))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--max-gap', type=float, default=10.0)
    ap.add_argument('--nav-vx', type=float, default=0.2)
    args = ap.parse_args()

    import rclpy
    from helix_arbiter.harness import Harness
    from helix_arbiter.trace import analyze

    bag = read_bag(args.bag)
    faults = bag['/helix/faults']
    t_first = faults[0][0]
    sched, t_prev, t_sched, gaps = [], t_first, 0.0, []
    for t, msg in faults:
        gap = t - t_prev
        if gap > args.max_gap:
            gaps.append({'bag_gap_s': round(gap, 3), 'replayed_as_s': args.max_gap})
            gap = args.max_gap
        t_sched += gap
        sched.append((t_sched, t - t_first, msg))
        t_prev = t

    rclpy.init()
    h = Harness(Path(tempfile.mkdtemp(prefix='s8_replay_')))
    try:
        h.start_arbiter()
        h.start_recovery()
        h.start_diagnosis()
        h.start_sink('dry_run')
        if not h.wait_for(lambda: any(s['reason'] == 'NO_LIVE_INPUT'
                                      for s in h.statuses()), 30):
            print('arbiter never became ready', file=sys.stderr)
            return 2
        nav = {'nav': (args.nav_vx, 0.0)}
        h.pump(1.0, nav)
        t0 = time.monotonic()
        for ts, _bag_t, msg in sched:
            wait = t0 + ts - time.monotonic()
            if wait > 0:
                h.pump(wait, nav)
            msg.timestamp = time.time()   # detector clock stamp, tracing only
            h.pub_fault.publish(msg)
        h.pump(6.0, nav)                   # let the last hold release
        h.pump(1.0)                        # then silence the source
    finally:
        h.shutdown()
        rclpy.shutdown()

    res = analyze(h.events)
    new_hints = [e for e in h.events if e['kind'] == 'hint']
    new_actions = [e for e in h.events if e['kind'] == 'action']
    orig_actions = [(round(t - t_first, 3), m.action, m.status, m.fault_id)
                    for t, m in bag['/helix/recovery_actions']]
    holds = []
    open_t = None
    for s in [e for e in h.events if e['kind'] == 'status']:
        if s['reason'] == 'HELIX_HOLD' and open_t is None:
            open_t = s['t']
        elif s['reason'] != 'HELIX_HOLD' and open_t is not None:
            holds.append(round(s['t'] - open_t, 3))
            open_t = None
    outs = [e for e in h.events if e['kind'] == 'output']
    sink_stop = [e for e in h.events if e['kind'] == 'sink' and e['api_id'] == 1003]
    sink_move = [e for e in h.events if e['kind'] == 'sink' and e['api_id'] == 1008]
    sha = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True,
                         text=True).stdout.strip()
    result = {
        'what': 'Session 8 real GO2 FaultEvents replayed through diagnosis->recovery->'
                'arbiter->sport sink(dry_run); off-robot, NOT hardware stopping evidence',
        'bag': 'Session 8 post_fix_demo (' + next(Path(args.bag).glob('*.db3')).name + ')',
        'bag_sha256_db3': hashlib.sha256(
            next(Path(args.bag).glob('*.db3')).read_bytes()).hexdigest(),
        'git_sha': sha,
        'faults_replayed': len(faults),
        'gap_compression': {'max_gap_s': args.max_gap, 'compressed': gaps},
        'fake_nav_vx': args.nav_vx,
        'original_session8': {'hints': len(bag['/helix/recovery_hints']),
                              'actions': orig_actions},
        'replay': {
            'hints': [(e['suggested_action'], e['fault_id'], e['rule']) for e in new_hints],
            'actions': [(e['action'], e['status'], e['fault_id']) for e in new_actions],
            'accepted_stops': sum(1 for e in new_actions if e['action'] == 'STOP_AND_HOLD'
                                  and e['status'] == 'ACCEPTED'),
            'hold_windows_s': holds,
            'output_msgs': len(outs),
            'nonfinite_outputs': sum(1 for o in outs if not o['finite']),
            'sink_stopmove_decisions': len(sink_stop),
            'sink_move_decisions_dry_run': len(sink_move),
        },
        'chains': res['chains'],
        'latency_summary_ms': res['summary'],
        'nonzero_outputs_during_hold': res['nonzero_outputs_during_hold'],
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, indent=1))
    with open(Path(args.out).with_suffix('.trace.jsonl'), 'w') as fp:
        for e in h.events:
            fp.write(json.dumps(e) + '\n')
    r = result['replay']
    print(json.dumps({k: result[k] for k in ('faults_replayed', 'latency_summary_ms',
                                             'nonzero_outputs_during_hold')}, indent=1))
    print('accepted stops', r['accepted_stops'], 'holds', r['hold_windows_s'])
    print('replay actions', r['actions'])
    complete = all(c['complete_to_output'] for c in res['chains'])
    print('all chains complete to output:', complete)
    return 0 if (complete and result['nonzero_outputs_during_hold'] == 0
                 and r['nonfinite_outputs'] == 0) else 1


if __name__ == '__main__':
    sys.exit(main())
