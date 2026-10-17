"""
C++ sink core vs helix_arbiter.sport_sink_core, event by event.

Random command/tick/response/state streams go through helix_sink_replay
and through the Python SinkLogic + SinkFeedback driven the same way; every
decision and every feedback field must match.
"""
import json
import os
import random
import subprocess

import pytest
from helix_arbiter.sport_sink_core import (
    MODE_DRY_RUN,
    MODES,
    SinkFeedback,
    SinkLogic,
)

REPLAY = os.environ.get('HELIX_SINK_REPLAY', '')


def python_lines(mode, events):
    logic, fb, req_id, out = SinkLogic(mode), SinkFeedback(), 1000, []
    for ev in events:
        kind, t = ev[0], ev[1]
        r = None
        if kind == 'cmd':
            r = logic.on_command(ev[2], ev[3], ev[4], t)
        elif kind == 'tick':
            r = logic.on_tick(t)
        elif kind == 'resp':
            fb.on_response(ev[2], ev[3], ev[4], t)
        elif kind == 'state':
            fb.on_state(ev[2], ev[3], ev[4], ev[5], t)
        if r is not None:
            req_id += 1
            if mode != MODE_DRY_RUN:
                fb.on_sent(req_id, r.api_id, r.reason, t)
            head = f'D {r.api_id} {r.reason} {r.parameter}'
        else:
            head = '-'
        out.append(
            f'{head} | {int(fb.stop_acked)} {fb.stop_ack_sent_at:.6f} {fb.stop_acked_at:.6f} '
            f'{int(fb.stopped)} {fb.stopped_since:.6f} {fb.last_api_id} '
            f'{fb.last_ack_request_id} {fb.last_ack_code} {int(fb.stopped_now(t))}')
    return out


def stream(seed, mode, n=600):
    """
    Build a random event stream.

    A shadow SinkLogic tracks the ids the sink will use, so responses can
    answer this sink's real requests as well as foreign ones.
    """
    rng = random.Random(seed)
    shadow, req_id, recent = SinkLogic(mode), 1000, []
    t, ev = 0.0, []
    speeds = [0.0, 0.0, 0.1, 0.15, -0.2, 0.25, 0.3, 0.9, float('nan')]
    for _ in range(n):
        t = round(t + rng.choice([0.001, 0.02, 0.05, 0.05, 0.1, 0.3]), 6)
        k = rng.random()
        r = None
        if k < 0.45:
            e = ('cmd', t, rng.choice(speeds), rng.choice([0.0, 0.0, 0.1]),
                 rng.choice([0.0, 0.0, 0.4, 0.6]))
            r = shadow.on_command(e[2], e[3], e[4], t)
        elif k < 0.6:
            e = ('tick', t)
            r = shadow.on_tick(t)
        elif k < 0.8 and recent and rng.random() < 0.7:
            rid, api = rng.choice(recent[-4:])
            e = ('resp', t, rid, rng.choice([api, api, api, 1002]), rng.choice([0, 0, 0, 3]))
        elif k < 0.8:
            e = ('resp', t, rng.randint(1, 999), 1003, 0)   # another publisher's reply
        else:
            e = ('state', t, rng.choice([0.0, 0.01, 0.04, 0.2]), rng.choice([0.0, 0.02]),
                 rng.choice([0.0, 0.05, 0.3]), rng.choice([1, 3]))
        if r is not None:
            req_id += 1
            recent.append((req_id, r.api_id))
        ev.append(e)
    return ev


def fmt(ev):
    def num(v):
        return 'nan' if v != v else repr(v)
    return ' '.join([ev[0]] + [num(x) if isinstance(x, float) else str(x) for x in ev[1:]])


@pytest.mark.skipif(not REPLAY, reason='HELIX_SINK_REPLAY not set')
@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('seed', range(12))
def test_cpp_core_matches_python(mode, seed):
    events = stream(seed, mode)
    text = '\n'.join(fmt(e) for e in events) + '\n'
    cpp = subprocess.run([REPLAY, mode], input=text, capture_output=True, text=True,
                         check=True).stdout.splitlines()
    py = python_lines(mode, events)
    assert len(cpp) == len(py)
    for i, (c, p) in enumerate(zip(cpp, py)):
        if c != p:
            # JSON payloads may differ only in number spelling, never in value
            ch, ct = c.split(' | ')
            ph, pt = p.split(' | ')
            assert ct == pt, f'event {i} {events[i]}: feedback {ct!r} != {pt!r}'
            cparts, pparts = ch.split(' ', 3), ph.split(' ', 3)
            assert cparts[:3] == pparts[:3], f'event {i} {events[i]}: {ch!r} != {ph!r}'
            assert json.loads(cparts[3]) == json.loads(pparts[3]), f'event {i}'


@pytest.mark.skipif(not REPLAY, reason='HELIX_SINK_REPLAY not set')
def test_streams_exercise_every_branch():
    seen = set()
    for mode in MODES:
        for seed in range(12):
            for line in python_lines(mode, stream(seed, mode)):
                head, tail = line.split(' | ')
                if head != '-':
                    seen.add(head.split(' ')[2])
                if tail.split(' ')[0] == '1':
                    seen.add('ACKED')
                if tail.split(' ')[3] == '1':
                    seen.add('STOPPED')
    assert {'MOVE', 'ZERO', 'DEADMAN', 'NOT_ARMED', 'REJECT_OVER_SINK_LIMIT',
            'REJECT_NON_FINITE', 'ACKED', 'STOPPED'} <= seen
