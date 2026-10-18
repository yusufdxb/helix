#!/usr/bin/env python3
"""Offline body-motion analysis from independent calibrated tracking.

No ROS, hardware I/O or control output. Input CSV: t_s,x_m,y_m,yaw_rad,valid.
Metadata JSON defines provenance, uncertainty and fault time in the same external
clock. This evaluates external measurement criteria only, never overall hardware
safety or permission to move. See docs/WALKING_STOP_EXPERIMENT.md.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def analyze(rows, meta):
    required = ('fault_t_s', 'sync_uncertainty_s', 'position_uncertainty_m',
                'yaw_uncertainty_rad', 'walking_cycles_before_fault')
    if any(k not in meta for k in required):
        raise ValueError('missing measurement metadata')
    if meta.get('tracking_source') not in ('external_camera', 'external_mocap', 'synthetic'):
        raise ValueError('tracking must be independent of GO2 estimates')
    for name in ('raw_tracking_sha256', 'calibration_sha256', 'gait_video_sha256'):
        value = meta.get(name, '')
        if len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
            raise ValueError('missing source artifact hash: ' + name)
    if not all(isinstance(meta[k], (int, float)) and math.isfinite(meta[k]) for k in required):
        raise ValueError('nonfinite measurement metadata')
    if not 0 <= meta['sync_uncertainty_s'] <= .02:
        raise ValueError('clock uncertainty must be bounded to at most 20 ms')
    if not 0 <= meta['position_uncertainty_m'] <= .002 or not 0 <= meta['yaw_uncertainty_rad'] <= .004:
        raise ValueError('tracking uncertainty exceeds the experiment budget')
    if len(rows) < 2 or any(not all(math.isfinite(r[k]) for k in ('t_s', 'x_m', 'y_m', 'yaw_rad')) for r in rows):
        raise ValueError('invalid tracking data')
    if any(b['t_s'] <= a['t_s'] for a, b in zip(rows, rows[1:])):
        raise ValueError('tracking timestamps must be strictly increasing')
    fault = meta['fault_t_s']
    window, still_hold = .2, .5
    values = []
    index = 0
    for i, b in enumerate(rows):
        while index + 1 < i and rows[index+1]['t_s'] <= b['t_s'] - window:
            index += 1
        a = rows[index]
        dt = b['t_s'] - a['t_s']
        if dt < window - 1e-8:
            continue
        segment = rows[index:i+1]
        valid = all(r['valid'] for r in segment) and all(
            y['t_s'] - x['t_s'] <= .05 for x, y in zip(segment, segment[1:]))
        speed = math.hypot(b['x_m'] - a['x_m'], b['y_m'] - a['y_m']) / dt
        yaw = abs(math.remainder(b['yaw_rad'] - a['yaw_rad'], 2*math.pi)) / dt
        position_error = 2*meta['position_uncertainty_m']/dt
        # A single window's net displacement can hide oscillating motion.
        # Check every sampled secant ending here with a 100-200 ms lag.
        secants = [(p, b['t_s']-p['t_s']) for p in segment if b['t_s']-p['t_s'] >= .1-1e-8]
        still_speed = max((math.hypot(b['x_m']-p['x_m'], b['y_m']-p['y_m']) +
                           2*meta['position_uncertainty_m'])/lag for p, lag in secants)
        still_yaw = max((abs(math.remainder(b['yaw_rad']-p['yaw_rad'], 2*math.pi)) +
                         2*meta['yaw_uncertainty_rad'])/lag for p, lag in secants)
        values.append(dict(t=b['t_s'], valid=valid, speed_low=max(0, speed-position_error),
                           speed_high=speed+position_error,
                           still_speed_high=still_speed, yaw_high=max(yaw, still_yaw)))
    # Require every sample in the pre-fault two-second interval to qualify.
    pre = [v for v in values if fault-2 <= v['t'] < fault]
    walking = (bool(pre) and pre[0]['t'] <= fault-2+.05 and pre[-1]['t'] >= fault-.05 and
               all(v['valid'] and v['speed_low'] >= .10 and v['speed_high'] <= .25 for v in pre) and
               meta['walking_cycles_before_fault'] >= 2)
    first_still = None
    proved_at = None
    # A missing frame must break a still interval even if a later window's
    # net displacement happens to be zero. Use the validity of every window.
    for v in (v for v in values if v['t'] >= fault):
        if v['valid'] and v['still_speed_high'] <= .05 and v['yaw_high'] <= .10:
            first_still = v['t'] if first_still is None else first_still
            if v['t'] - first_still >= still_hold:
                proved_at = v['t']
                break
        else:
            first_still = None
    latency = None if proved_at is None else first_still - fault
    post = [v for v in values if first_still is not None and v['t'] >= first_still]
    held_still = bool(proved_at is not None and rows[-1]['t_s'] >= fault+3 and
                      all(v['valid'] and v['still_speed_high'] <= .05 and v['yaw_high'] <= .10 for v in post))
    u = meta['sync_uncertainty_s']
    return dict(evidence_class='Synthetic' if meta['tracking_source'] == 'synthetic' else 'HW-external-input',
                tracking_provenance_verified_by_tool=False,
                steady_walking_criteria_met=bool(walking), externally_still=proved_at is not None,
                remained_still_through_observation=held_still,
                external_stop_latency_s=latency,
                latency_interval_s=None if latency is None else [max(0, latency-u), latency+u],
                still_confirmed_at_s=proved_at, velocity_window_s=window, still_hold_s=still_hold,
                external_walking_stop_criteria_met=bool(walking and held_still and latency is not None and latency+u <= 1.5),
                overall_hardware_trial_verdict='NOT_EVALUATED', motion_authorized=False,
                notes=['Tracking and gait annotations require independent provenance review.',
                       'The 200 ms trailing displacement window delays the still criterion.',
                       'Sampled secants do not bound unobserved between-frame motion or establish actuator torque removal.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track', required=True, type=Path)
    parser.add_argument('--metadata', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    with args.track.open() as f:
        rows = [dict(t_s=float(r['t_s']), x_m=float(r['x_m']), y_m=float(r['y_m']),
                     yaw_rad=float(r['yaw_rad']), valid=r['valid'] == '1') for r in csv.DictReader(f)]
    result = analyze(rows, json.loads(args.metadata.read_text()))
    result['analyzed_csv_sha256'] = hashlib.sha256(args.track.read_bytes()).hexdigest()
    result['metadata_sha256'] = hashlib.sha256(args.metadata.read_bytes()).hexdigest()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return 0 if result['external_walking_stop_criteria_met'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
