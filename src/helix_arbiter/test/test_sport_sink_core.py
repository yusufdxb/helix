"""Pure tests of the GO2 sport sink decision logic (no ROS, no unitree_api)."""
import json
import math
from types import SimpleNamespace as NS

import pytest
from helix_arbiter.sport_sink_core import (
    API_MOVE,
    API_STOP_MOVE,
    MODE_ARMED,
    MODE_DRY_RUN,
    MODE_STOP_ONLY,
    SinkLogic,
    SportRequest,
    fill_unitree_request,
)


def test_zero_sends_stopmove_once_then_rate_limited():
    s = SinkLogic(MODE_ARMED)
    assert s.on_command(0, 0, 0, 0.0).api_id == API_STOP_MOVE
    assert s.on_command(0, 0, 0, 0.02) is None
    assert s.on_command(0, 0, 0, 0.6).api_id == API_STOP_MOVE   # 2 Hz repeat


def test_armed_move_payload():
    s = SinkLogic(MODE_ARMED)
    r = s.on_command(0.15, 0.0, 0.1, 0.0)
    assert r.api_id == API_MOVE
    assert json.loads(r.parameter) == {'x': 0.15, 'y': 0.0, 'z': 0.1}


def test_dry_run_decides_like_armed():
    a, d = SinkLogic(MODE_ARMED), SinkLogic(MODE_DRY_RUN)
    seq = [(0.2, 0.0), (0.2, 0.06), (0.0, 0.1), (0.9, 0.2), (0.1, 0.3)]
    assert [a.on_command(v, 0, 0, t) for v, t in seq] == \
        [d.on_command(v, 0, 0, t) for v, t in seq]


def test_stop_only_never_moves():
    s = SinkLogic(MODE_STOP_ONLY)
    for i in range(100):
        r = s.on_command(0.2, 0.0, 0.0, i * 0.02)
        assert r is None or r.api_id == API_STOP_MOVE


def test_moving_then_zero_stops_immediately():
    s = SinkLogic(MODE_ARMED)
    s.on_command(0.2, 0, 0, 0.0)
    r = s.on_command(0.0, 0, 0, 0.01)          # transition: not rate limited
    assert r.api_id == API_STOP_MOVE and r.reason == 'ZERO'


def test_over_limit_stops_not_clamps():
    s = SinkLogic(MODE_ARMED)
    s.on_command(0.2, 0, 0, 0.0)
    r = s.on_command(0.9, 0, 0, 0.01)
    assert r.api_id == API_STOP_MOVE and r.reason == 'REJECT_OVER_SINK_LIMIT'


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_non_finite_stops(bad):
    s = SinkLogic(MODE_ARMED)
    s.on_command(0.2, 0, 0, 0.0)
    assert s.on_command(bad, 0, 0, 0.001).api_id == API_STOP_MOVE


def test_deadman_on_input_silence():
    s = SinkLogic(MODE_ARMED, input_timeout_sec=0.25)
    s.on_command(0.2, 0, 0, 0.0)
    assert s.on_tick(0.2) is None
    r = s.on_tick(0.26)
    assert r.api_id == API_STOP_MOVE and r.reason == 'DEADMAN'
    assert s.on_tick(0.3) is None                 # rate limited
    assert s.on_tick(0.8).api_id == API_STOP_MOVE


def test_deadman_before_any_input():
    s = SinkLogic(MODE_ARMED)
    assert s.on_tick(0.0).reason == 'DEADMAN'


def test_move_rate_limited():
    s = SinkLogic(MODE_ARMED, move_hz=20.0)
    assert s.on_command(0.2, 0, 0, 0.0).api_id == API_MOVE
    assert s.on_command(0.2, 0, 0, 0.02) is None
    assert s.on_command(0.2, 0, 0, 0.05).api_id == API_MOVE


def test_fill_request_matches_field_notes_and_refuses_damp():
    msg = NS(header=NS(identity=NS(id=None, api_id=None), lease=NS(id=None),
                       policy=NS(priority=None, noreply=None)), parameter=None)
    fill_unitree_request(msg, SportRequest(API_STOP_MOVE, '', 'x'), 7)
    assert (msg.header.identity.id, msg.header.identity.api_id) == (7, 1003)
    assert msg.header.lease.id == 0 and msg.header.policy.noreply is False
    with pytest.raises(ValueError):
        fill_unitree_request(msg, SportRequest(1001, '', 'damp'), 8)


def test_bad_mode_refused():
    with pytest.raises(ValueError):
        SinkLogic('normal')
