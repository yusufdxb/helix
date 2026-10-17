"""OverspeedGuard (scripts/hw/stop_guard.py) against the 2026-10-17 GO2 bags.

The fixture holds every /sportmodestate and /utlidar/robot_odom speed sample
(and SportModeState position) from 0.1 s before the first Move to 4 s after
it, for the four motion runs. It was extracted with
scripts/hw/stop_guard_fixture.py from hardware_eval_20261017/native_stop.
"""
import json
import math
from pathlib import Path

import pytest

from scripts.hw.stop_guard import OverspeedGuard

FIXTURE = Path(__file__).parent / 'fixtures' / 'stop_guard_20261017.json'
RUNS = json.loads(FIXTURE.read_text())
STALLED = '20261017T222205_stop_motion'
PASS_1 = '20261017T222412_stop_motion'
PASS_2 = '20261017T223203_stop_motion'
ABORTED = '20261017T223245_stop_motion'
PERIOD = 1.0 / 300.0


def samples(run):
    """Merged (t_s, kind, values) in time order, as the runner receives them."""
    out = [(s[0] / 1000.0, 'state', s[1:]) for s in RUNS[run]['state']]
    out += [(o[0] / 1000.0, 'odom', o[1:]) for o in RUNS[run]['odom']]
    return sorted(out, key=lambda e: e[0])


def replay(run, guard, mutate=None):
    for t, kind, vals in samples(run):
        if t < 0.0:
            continue
        if mutate is not None:
            vals = mutate(t, kind, list(vals))
        guard.speed(t, vals[0], kind)
        if kind == 'state':
            guard.position(t, vals[1], vals[2])
        guard.check_stale(t)
    return guard


def test_the_old_single_sample_rule_tripped_on_run_223245():
    peak = max(s[1] for s in RUNS[ABORTED]['state'] if s[0] >= 0.0)
    assert peak > 0.40
    above = [s for s in RUNS[ABORTED]['state'] if s[1] > 0.40]
    assert len(above) == 1, 'the onset transient is one SportModeState sample'


def test_the_two_topics_are_one_estimate():
    """Odometry is every other SportModeState sample, so it cannot confirm it."""
    state = {round(s[0]): s[1] for s in RUNS[PASS_1]['state']}
    diffs = [abs(o[1] - state[round(o[0])]) for o in RUNS[PASS_1]['odom']
             if round(o[0]) in state]
    diffs.sort()
    assert len(diffs) > 100
    assert diffs[len(diffs) // 2] == 0.0


def test_guard_ignores_the_onset_transient_of_run_223245():
    g = replay(ABORTED, OverspeedGuard(0.40))
    assert not g.tripped, g.detail


@pytest.mark.parametrize('run,abort', [(PASS_1, 0.40), (PASS_2, 0.40), (STALLED, 0.35)])
def test_guard_stays_quiet_on_the_recorded_runs(run, abort):
    g = replay(run, OverspeedGuard(abort))
    assert not g.tripped, g.detail


def test_one_sample_above_the_hard_limit_trips_at_once():
    def spike(t, kind, vals):
        if kind == 'state' and abs(t - 0.200) < PERIOD:
            vals[0] = 0.70
        return vals
    g = replay(PASS_1, OverspeedGuard(0.40), mutate=spike)
    assert g.reason == 'hard'
    assert abs(g.detail['t'] - 0.200) < PERIOD


def test_sustained_overspeed_trips_within_persist_plus_one_period():
    g = OverspeedGuard(0.40, persist_s=0.03)
    t, onset = 0.0, 0.5
    while not g.tripped and t < 2.0:
        g.speed(t, 0.50 if t >= onset else 0.20, 'state')
        t += PERIOD
    assert g.reason == 'overspeed'
    assert 0.03 <= g.detail['t'] - onset <= 0.03 + PERIOD + 1e-9


def test_recorded_run_with_sustained_overspeed_trips():
    def push(t, kind, vals):
        if 1.0 <= t <= 1.2:
            vals[0] = 0.50
        return vals
    g = replay(PASS_1, OverspeedGuard(0.40), mutate=push)
    assert g.reason == 'overspeed'
    assert g.detail['t'] - 1.0 <= 0.03 + 2 * PERIOD


def test_odometry_alone_can_trip():
    g = OverspeedGuard(0.40)
    for i in range(40):
        g.speed(i * 2 * PERIOD, 0.50, 'odom')
        g.speed(i * 2 * PERIOD + PERIOD, 0.10, 'state')
    assert g.reason == 'overspeed'
    assert g.detail['source'] == 'odom'


def test_a_dip_resets_persistence():
    g = OverspeedGuard(0.40, persist_s=0.03)
    t = 0.0
    for _ in range(60):
        g.speed(t, 0.45, 'state')
        g.speed(t + PERIOD, 0.45, 'state')
        g.speed(t + 2 * PERIOD, 0.10, 'state')
        t += 3 * PERIOD
    assert not g.tripped


def test_travel_limit():
    g = OverspeedGuard(0.40, max_travel_m=0.60)
    for i in range(100):
        g.position(i * 0.01, 0.007 * i, 0.0)
    assert g.reason == 'travel'
    assert g.detail['travel_m'] > 0.60


def test_silent_state_topic_trips_while_armed():
    g = OverspeedGuard(0.40, stale_s=0.20)
    g.arm(0.0)
    g.speed(0.10, 0.1, 'state')
    assert g.check_stale(0.25) is None
    assert g.check_stale(0.31) == 'stale'


def test_never_a_sample_trips_once_armed():
    g = OverspeedGuard(0.40, stale_s=0.20)
    g.arm(0.0)
    assert g.check_stale(0.21) == 'stale'


def test_nonfinite_speed_trips():
    g = OverspeedGuard(0.40)
    g.speed(0.0, math.nan, 'state')
    assert g.reason == 'nonfinite'


def test_first_reason_is_kept():
    g = OverspeedGuard(0.40)
    g.speed(0.0, 0.9, 'state')
    g.speed(0.1, math.nan, 'odom')
    assert g.reason == 'hard'


@pytest.mark.parametrize('abort,hard', [(0.0, 0.6), (0.7, 0.6), (-1.0, 0.6)])
def test_invalid_limits_are_refused(abort, hard):
    with pytest.raises(ValueError):
        OverspeedGuard(abort, hard_limit=hard)
