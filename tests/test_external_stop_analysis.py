"""Synthetic measurement regressions, never hardware stopping evidence."""
import math

import pytest

from scripts.hw.external_stop_analysis import analyze


def fixture():
    rows = [dict(t_s=i/120, x_m=.2*min(i/120, 3.2), y_m=0., yaw_rad=0., valid=True)
            for i in range(841)]
    meta = dict(tracking_source='synthetic', fault_t_s=3., sync_uncertainty_s=.015,
                position_uncertainty_m=.002, yaw_uncertainty_rad=.004,
                walking_cycles_before_fault=2, raw_tracking_sha256='a'*64,
                calibration_sha256='b'*64, gait_video_sha256='c'*64)
    return rows, meta


def test_independent_measurement_reports_conservative_window_and_uncertainty():
    r = analyze(*fixture())
    assert r['steady_walking_criteria_met']
    assert r['externally_still'] and r['external_walking_stop_criteria_met']
    assert .3 < r['external_stop_latency_s'] < .5
    lo, hi = r['latency_interval_s']
    assert hi-lo == pytest.approx(.030)
    assert r['evidence_class'] == 'Synthetic'
    assert r['overall_hardware_trial_verdict'] == 'NOT_EVALUATED'
    assert not r['motion_authorized']


def test_body_shift_and_return_cannot_qualify_as_steady_walking():
    rows, meta = fixture()
    for r in rows:
        r['x_m'] = .04*math.sin(r['t_s']*math.pi) if r['t_s'] < 3 else 0
    assert not analyze(rows, meta)['steady_walking_criteria_met']


def test_missing_gait_annotation_cannot_qualify_velocity_as_walking():
    rows, meta = fixture()
    meta['walking_cycles_before_fault'] = 0
    assert not analyze(rows, meta)['external_walking_stop_criteria_met']


def test_state_estimator_is_not_an_independent_tracking_source():
    rows, meta = fixture()
    meta['tracking_source'] = 'sportmodestate'
    with pytest.raises(ValueError, match='independent'):
        analyze(rows, meta)


def test_no_stop_and_acknowledgment_cannot_create_external_stillness():
    rows, meta = fixture()
    for r in rows:
        r['x_m'] = .2*r['t_s']
    meta['stop_acked'] = True
    assert not analyze(rows, meta)['externally_still']


def test_window_period_oscillation_cannot_be_hidden_by_zero_net_displacement():
    rows, meta = fixture()
    for r in rows:
        if r['t_s'] > 3.2:
            r['x_m'] += .02*math.sin(2*math.pi*5*(r['t_s']-3.2))
    assert not analyze(rows, meta)['external_walking_stop_criteria_met']


@pytest.mark.parametrize('kind', ['invalid_frames', 'gap', 'rotation', 'resumed_motion'])
def test_unobserved_or_continued_motion_fails_the_held_stop(kind):
    rows, meta = fixture()
    if kind == 'gap':
        rows = [r for r in rows if not 4 < r['t_s'] < 4.3]
    for r in rows:
        if kind == 'invalid_frames' and r['t_s'] > 4:
            r['valid'] = False
        if kind == 'rotation':
            r['yaw_rad'] = .3*r['t_s']
        if kind == 'resumed_motion' and r['t_s'] > 5:
            r['x_m'] += .2*(r['t_s']-5)
    assert not analyze(rows, meta)['external_walking_stop_criteria_met']


@pytest.mark.parametrize('key,value', [('sync_uncertainty_s', .021),
                                     ('position_uncertainty_m', .003),
                                     ('yaw_uncertainty_rad', math.nan)])
def test_uncertainty_budget_is_required(key, value):
    rows, meta = fixture()
    meta[key] = value
    with pytest.raises(ValueError):
        analyze(rows, meta)


def test_duplicate_camera_timestamps_rejected():
    rows, meta = fixture()
    rows[4]['t_s'] = rows[3]['t_s']
    with pytest.raises(ValueError):
        analyze(rows, meta)
