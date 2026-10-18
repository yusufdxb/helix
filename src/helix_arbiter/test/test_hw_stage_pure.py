"""Pure tests for the stage runner's gating and stop metrics."""
import json

from helix_arbiter.hw_stage import (
    FAIL,
    PASS,
    config_hash,
    gate,
    speeds_from_odom,
    stop_metrics,
)


def _write(session, stage, **ev):
    d = session / f'stage_{stage}'
    d.mkdir(parents=True, exist_ok=True)
    (d / 'evidence.json').write_text(json.dumps(ev))


def test_stage_a_needs_no_predecessor(tmp_path):
    assert gate('A', tmp_path, 'sha', 'h') is None


def test_cannot_skip_a_stage(tmp_path):
    _write(tmp_path, 'A', verdict=PASS, git_sha='sha', config_hash='h', rehearsal=False)
    assert 'stage C evidence missing' not in (gate('C', tmp_path, 'sha', 'h') or '')
    assert gate('C', tmp_path, 'sha', 'h') is not None      # B missing


def test_failed_predecessor_blocks(tmp_path):
    _write(tmp_path, 'D', verdict=FAIL, git_sha='sha', config_hash='h', rehearsal=False)
    assert 'not PASS' in gate('E', tmp_path, 'sha', 'h')


def test_sha_or_config_change_blocks(tmp_path):
    _write(tmp_path, 'D', verdict=PASS, git_sha='sha', config_hash='h', rehearsal=False)
    assert gate('E', tmp_path, 'sha', 'h') is None
    assert 'SHA' in gate('E', tmp_path, 'other', 'h')
    assert 'config' in gate('E', tmp_path, 'sha', 'h2')


def test_rehearsal_evidence_never_unlocks_hardware(tmp_path):
    _write(tmp_path, 'D', verdict=PASS, git_sha='sha', config_hash='h', rehearsal=True)
    assert 'rehearsal' in gate('E', tmp_path, 'sha', 'h')


def test_config_hash_sensitive_to_params():
    a = config_hash({'f': 'x'}, {'arbiter': {'hold_timeout_sec': 0.5}})
    b = config_hash({'f': 'x'}, {'arbiter': {'hold_timeout_sec': 0.6}})
    assert a != b and a == config_hash({'f': 'x'}, {'arbiter': {'hold_timeout_sec': 0.5}})


def _odom(profile, dt=1 / 150):
    out, x = [], 0.0
    for i, v in enumerate(profile):
        x += v * dt
        out.append({'t': i * dt, 'x': x, 'y': 0.0, 'vx': v, 'vy': 0.0, 'wz': 0.0})
    return speeds_from_odom(out)


def test_stop_metrics_time_and_distance():
    prof = [0.15] * 150 + [0.15 * (0.9 ** k) for k in range(150)]
    m = stop_metrics(_odom(prof), t_hold=1.0)
    assert 0.1 < m['stop_time_s'] < 0.5
    assert 0.0 < m['stop_distance_m'] < 0.05
    assert abs(m['speed_at_hold'] - 0.15) < 0.01


def test_never_stopping_reports_none():
    m = stop_metrics(_odom([0.15] * 300), t_hold=1.0)
    assert m['stop_time_s'] is None
