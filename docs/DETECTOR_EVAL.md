# Detector evaluation (native AnomalyCore)

This document records how the native detector configuration shipped in
`helix_native.launch.py` (`DETECTOR_OVERRIDES`) was chosen, and how it scored
on two GO2 captures that played no part in choosing it.

Script: [`analysis/detector_eval.py`](../analysis/detector_eval.py). Every
number it reports comes from `helix_detector_eval`, which runs the same
`AnomalyCore` (`src/helix_sensing_cpp/src/anomaly_core.cpp`) as the C++ node.
Nothing in the script re-implements detection.

## What was wrong with the shipped detector

The shipped parameters (`zscore_threshold 4.0`, `window_size 60`) detect
sustained faults, but they cannot keep a fault active. The rolling
reference window absorbs the fault: after a few seconds the z-score falls
back under threshold and the detector emits a clear record while the fault
is still present. This is the self-masking that
[`analysis/README.md`](../analysis/README.md) describes. Scored the way the
supervisor now consumes the detector (a fault holds until its clear record),
self-masking becomes a false clear. A false clear would release a hold while
the robot's LiDAR is still degraded.

The DEV report shows this on the shipped point:

| Fault model (539 onsets each) | Detected | False clears while the fault is present |
|---|---|---|
| 50 % rate collapse | 539 | **539** |
| 75 % rate collapse | 539 | **539** |
| 70 % drift over 60 s | 317 | **317** |
| 70 % drift over 300 s | 93 / 392 | **93** |
| topic death | 539 | 0 |

## Fixes (all in `AnomalyCore`, all off by default, so Python parity holds)

| Parameter | Effect |
|---|---|
| `freeze_baseline` | samples stop entering the reference window while a violation streak or an active fault is present, so a fault cannot become its own baseline |
| `std_floor_rel` | floors the window std at a fraction of the window mean; a perfectly flat window (a 1.0 Hz topic) stays evaluable instead of being skipped |
| `duration_tolerance_s` | the duration gate allows for the 0.5 s sample grid (the cause of the live-vs-replay gnss disagreement in `analysis/README.md` section 7) |
| `clear_samples`, `clear_zscore` | a fault clears only after N consecutive normal samples; the clear is published on `/helix/fault_clear` |
| `anchor_tolerance_rel` | a long-term anchor taken from a full, fault-free window; a value more than this fraction away from it is a violation even when the z-score has adapted (slow drift). The clear needs half the band (hysteresis) |
| `anchor_metric_prefix` | the anchor applies only to `rate_hz/` metrics; on `pose/displacement_rate_m_s` it flooded the log (588 records on one capture) |

The native rate monitor also adds `stale_after_s` (1.0 s in the native
launch): a topic with no arrival for 1 s reports NaN at once. The legacy
monitor kept a near-normal rate until its 5 s window emptied.

Unit tests: `src/helix_sensing_cpp/test/test_anomaly_core.cpp` (7 cases that
use the shipped native parameters, including slow drift and the half-band
clear) and `test_rate_window.cpp` (7). The hysteresis test was
mutation-checked: removing the half-band rule makes it fail.

## Protocol

- Real GO2 `/helix/metrics` streams are the input. False alarms are counted on
  the unmodified streams.
- Detection injects a synthetic onset into a real stream: one stop-relevant
  rate metric is transformed the way the 5 s / 0.5 s rate estimator would
  report the fault, and the real noise is kept. Models: `collapse_50`,
  `collapse_75`, `death`, `death_s1` (death with the 1 s stale rule),
  `drift_70` (to 70 % over 60 s) and `slowdrift_70` (to 70 % over 300 s,
  scored on captures long enough to hold one).
- Recovery runs the same faults for 20 s, then the real stream resumes. It
  measures clear latency and false clears.
- Selection uses DEV only. Hard constraints: zero stop-metric false alarms
  and zero false clears. Then the highest mean TPR, then the lowest latency,
  then the fewest log-only false alarms.
- A capture that has been used to judge a configuration is DEV from then on.

## Round 1 (selected on April only)

DEV was three April captures: 901 s, robot idle. The chosen point was
`freeze_z4.0_w120_f0.01_d1.0_a0.0`.

| Held-out capture | Stop false alarms |
|---|---|
| 2026-10-06 loaded standing, 1103 s | 0 |
| 2026-10-17 lying, Python baseline run, 1800 s | **2** (`rate_hz/utlidar_cloud` stalls), 2.48 / h |

**Round 1 failed held-out.** Both October captures then moved into DEV.

## Round 2 (selected on April + both October captures)

DEV was five captures, 3805 s (1.057 h). Reproduce it with:

```bash
colcon build --packages-select helix_msgs helix_sensing_cpp
python3 analysis/detector_eval.py --eval-bin build/helix_sensing_cpp/helix_detector_eval \
  --dev oct06_loaded_standing=hardware_eval_20261006/bags/loaded_standing_observe \
  --dev oct17_lying_pybaseline=hardware_eval_20261017/pybaseline/bag \
  --out analysis/results/detector_eval_round2_dev.json
```

At commit e4f3032 this run took 7 min 40 s on the workstation. Its output is
byte-identical to [`analysis/results/detector_eval_round2_dev.json`](../analysis/results/detector_eval_round2_dev.json)
(110 configurations).

Chosen: `freeze_z4.0_w60_f0.02_d1.0_a0.1` (z 4.0, window 60, trigger 3,
duration 1.0 s with 0.25 s tolerance, cooldown 1.0 s, freeze, std floor 2 %,
clear after 4 samples, anchor 10 % on `rate_hz/`).

| DEV result | shipped z4/w60 | proposed z3.5/w120 | **chosen** |
|---|---|---|---|
| Stop-metric false alarms | 0 | 3 (2.84 / h) | **0** |
| 50 % collapse detected | 539/539 | 535/539 | **539/539** |
| 75 % collapse detected | 539/539 | 535/539 | **539/539** |
| Topic death detected | 539/539 | 535/539 | **539/539** |
| 70 % drift / 60 s detected | 317/539 | 530/539 | **539/539** |
| 70 % drift / 300 s detected | 93/392 | 179/392 | **392/392** |
| False clears during a fault, all models | 1488 | 1779 | **0** |
| Recovery: clears after the fault ends (collapse_50) | 0/539 | 0/539 | **539/539** |

The table counts false clears over the six detection models.

Chosen-point median latencies (from onset; one value per capture that
contains the metric):

| Model | Detection latency | Clear latency |
|---|---|---|
| 50 % collapse | 2.0 s | 1.5 to 2.0 s |
| 75 % collapse | 3.0 s | |
| Topic death, legacy 5 s window | 5.5 to 6.0 s | |
| Topic death, 1 s stale rule | 2.0 to 2.5 s | 1.5 to 2.0 s |
| 70 % drift over 60 s | 21.0 to 21.5 s | |
| 70 % drift over 300 s | 101.0 to 101.5 s | |

Drift is caught once the rate is about 10 % below the anchor: about 21 s
into the 60 s ramp and about 101 s into the 300 s ramp.

### Round 2 held-out (2026-10-17, passed)

Held-out captures: the two native-profile runs recorded after the
configuration was frozen at 1254bbe. Both were scored once, with no
re-selection:

| Capture | Duration | Robot |
|---|---|---|
| `hardware_eval_20261017/native_partial/bag` | 176.5 s | standing |
| `hardware_eval_20261017/native/bag` | 600.0 s | lying |

Together they cover 0.216 h. Reproduce with:

```bash
python3 analysis/detector_eval.py --eval-bin build/helix_sensing_cpp/helix_detector_eval \
  --dev oct06_loaded_standing=hardware_eval_20261006/bags/loaded_standing_observe \
  --dev oct17_lying_pybaseline=hardware_eval_20261017/pybaseline/bag \
  --heldout oct17_standing_native_175s=hardware_eval_20261017/native_partial/bag \
  --heldout oct17_lying_native_600s=hardware_eval_20261017/native/bag \
  --out analysis/results/detector_eval_round2_heldout.json
```

The DEV section of that output is byte-identical to
`detector_eval_round2_dev.json`.

| Held-out result | shipped z4/w60 | proposed z3.5/w120 | **chosen** |
|---|---|---|---|
| Stop-metric false alarms | 0 | **1** (4.64 / h) | **0** |
| Log-only false alarm episodes | 0 | 2 (`gnss`, `multiplestate`) | **0** |
| 50 % collapse detected | 120/120 | 118/120 | **120/120** |
| 75 % collapse detected | 120/120 | 118/120 | **120/120** |
| Topic death detected (5 s window / 1 s stale) | 120/120, 120/120 | 118/120, 118/120 | **120/120, 120/120** |
| 70 % drift / 60 s detected | 88/120 | 117/120 | **120/120** |
| 70 % drift / 300 s detected | 8/44 | 31/44 | **44/44** |
| False clears during a fault, all models | 336 | 384 | **0** |
| Recovery: clears after the fault ends (collapse_50, death_s1) | 0/120, 120/120 | 0/120, 118/120 | **120/120, 120/120** |

The chosen point's held-out latencies match DEV:

| Model | Detection latency | Clear latency |
|---|---|---|
| Collapse 50 % | 2.0 s | 1.5 s |
| Collapse 75 % | 3.0 s | |
| Death, 5 s window | 5.5 to 6.0 s | |
| Death, 1 s stale | 2.0 to 2.5 s | 1.5 s |
| Drift over 60 s | 21.5 s | |
| Drift over 300 s | 101.5 s | |

The proposed point failed held-out again, as it did in round 1: one
`rate_hz/utlidar_cloud` stop false alarm in the 10-minute lying capture.

### What round 2 does not show

- **A short held-out set.** 0.216 h with 0 stop false alarms gives a 95 %
  Poisson upper bound of about 13.9 per hour. That is a pass on fresh data,
  not a tight rate. Any later capture used to change the configuration
  becomes DEV.
- **The held-out captures have no pose metrics.** The native profile does
  not run the pose-drift monitor. So the log-only noise on
  `pose/displacement_rate_m_s` (next bullet) was not exercised held-out.
- **Log-only noise is not zero.** The chosen point raises non-stop faults on
  `pose/displacement_rate_m_s`: 19 episodes (275 records) over the 30 min
  lying capture, plus 6 records on `rate_hz/multiplestate`. These faults
  never assert a hold: `pose/` and `multiplestate` match no
  `stop_metric_prefixes`. They still fill the audit trail. On a lying robot
  the displacement rate sits near zero, so a floor relative to the mean has
  almost nothing to work with. An absolute floor is the likely fix; it is
  not done.
- **Synthetic onsets.** Detection uses synthetic onsets on real noise. No
  physical LiDAR fault (occlusion, unplug) has been scored with this
  configuration.
- **Idle and standing only.** No capture with the robot walking is in DEV.
  Rates under locomotion load are unmeasured.
- **One Poisson bound.** 0 stop false alarms in 1.057 h gives a 95 % upper
  bound of about 2.8 per hour.
