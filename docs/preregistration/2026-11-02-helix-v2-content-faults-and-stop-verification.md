# Pre-registration: HELIX v2 content-fault detection and independent stop verification

Registered 2026-11-02, before any detection or verification result was computed.
Only data inventory (topic counts, message decoding, command counts) has been looked at.

## Claims

**C1 (primary).** On held-out GO2 recordings with message-level content faults injected
mid-recording into `/utlidar/imu` (frozen republish, gyro bias step 0.2 rad/s, gyro scale x0.5,
z-axis sign flip), the HELIX v2 monitor detects at least 0.5 more of the fault episodes within
2.0 s of onset (absolute detection-rate difference) than the strongest conventional comparator,
the union of a FrequencyStatus-style rate band with timeout (B-rate) and a per-channel value
range check (B-range), while holding a held-out clean-data false-alarm rate of at most 2.0 per hour.

**C2.** On held-out GO2 walk-to-StopMove events with an injected "estimator lies" fault (the
reported body velocity drops to zero while the robot keeps walking), the HELIX v2 StopVerifier
wrongly reports VERIFIED in at most 0.10 of events, against approximately 1.0 for a single-source
estimator check (B-est), while reporting VERIFIED for at least 0.90 of genuine held-out stops.

C2 is near-structural for B-est by design. The falsifiable parts are the HELIX genuine-stop
VERIFIED rate on real data and the wrong-VERIFIED rate under the lie. C1 is near-structural for
B-rate on content faults (rate is unchanged). The falsifiable parts are the false-alarm rate on
held-out walking and standing data and detection against B-range during real motion.

## Data and split (recording level, fixed now)

| Use | Recordings | Content |
|---|---|---|
| Detection DEV | odin-hw-20260901-1926, odin-p7 G1_varied_speed, G1_yaw_in_place, G1_negative_stationary, G1_forward_only (about 212 s) | body IMU (`/sportmodestate`, `/lowstate`) plus LiDAR IMU |
| Detection HELD-OUT | odin-hw-standing-20260901-1935, odin-p7 G2_prelim_asymmetric_drag, G1_stride_segmentation (about 276 s) | same |
| Clean FA DEV (checks without LiDAR IMU) | PHM 10-17 motion, motion2 | `/sportmodestate`, `/lowstate` |
| Clean FA HELD-OUT | PHM 10-17 motion3, motion4, fs_standing | same |
| Stop DEV | PHM 10-17 faultwalk1 (10 walk-to-StopMove events) | sport, lowstate, `/api/sport/request` |
| Stop HELD-OUT | PHM 10-17 motion4 (16), partialwalk1 (20): 36 events | same |
| Stop, single-source only | hardware_eval_20261017/native_stop/*_motion (4 events, no `/lowstate`) | expected verdict UNVERIFIED |

faultwalk1 and partialwalk1 contain whatever physical condition the PHM protocol applied.
partialwalk1 and faultwalk1 are not used as clean-FA data; partialwalk1 contributes stop events only.

## Arms

### C1 detection

| Arm | Purpose |
|---|---|
| Treatment: HELIX v2 | timeout + rate band (identical to B-rate) + frozen-content + stamp-stall + body/LiDAR gyro residual (rotation and thresholds from DEV) |
| No-treatment control | clean held-out runs (no injection) for every arm: false alarms per hour |
| Comparator B-rate | windowed rate outside a [min,max] band, plus timeout. Band tolerance and timeout from DEV |
| Comparator B-range | per-channel value bounds from DEV (min/max expanded by 10% of span) |
| Comparator union (primary) | alarm if B-rate or B-range alarms |
| Sham | residual check with the LiDAR stream circularly shifted by 20 s, calibrated identically on DEV (same statistics, alignment information removed) |
| Oracle | per-class threshold tuned on held-out subject to held-out FA at most 2/h (ceiling of the check family) |
| Positive controls | F1 dropout must be detected by B-rate and HELIX (at least 0.95); bias 1.0 rad/s must be detected by HELIX (at least 0.95) |
| Ablation (A4) | per-check contribution per fault class; HELIX minus B-rate differs only by frozen, stamp, residual |
| Secondary: B-z | existing AnomalyCore chosen config `freeze_z4.0_w60_f0.02_d1.0_a0.1` on the 1 s rate metric, F1 and F2 only |

Fault classes, all injected into `/utlidar/imu` at onsets on a fixed grid (every 6 s, from 10 s after
recording start to 5 s before end), persisting to end of run, scored in [onset, onset + 2.0 s]:
F1 dropout, F2 rate collapse (drop every other message), F3a frozen payload and stamp, F3b frozen
payload with advancing stamp, F4 gyro z bias +0.2 rad/s (secondary +0.1), F5 gyro scale x0.5,
F6 gyro z sign flip, F7 gyro noise burst sigma 0.3 rad/s. Primary pool: F3a, F4(0.2), F5, F6.
Frozen and dropout are also injected into `/sportmodestate` on PHM held-out walks (secondary).

### C2 stop verification

| Arm | Purpose |
|---|---|
| Treatment: HELIX StopVerifier | evidence groups {estimator velocity}, {joint encoder dq}, {inertial: body IMU gyro and accel}. VERIFIED needs at least 2 groups quiet including a raw-sensor group and no group moving, within a 2.0 s deadline. FAILED if any group still moving at deadline. UNVERIFIED if fewer than 2 groups available or disagreement without motion evidence |
| No-treatment control: B-ack | command acknowledged = stopped (always VERIFIED), what HELIX v1 and most stacks do |
| Comparator B-est | estimator speed < 0.05 m/s and yaw rate < 0.1 rad/s sustained 0.5 s within the 2.0 s deadline |
| Sham | joint dq and IMU groups replaced by data from a randomly chosen genuine post-stop standing window (seeded); must be fooled like B-est |
| Oracle | ground-truth label by construction |
| Positive control | non-compliance splice (all streams after the stop replaced by the walking segment before it): both B-est and HELIX must FAIL at least 0.95 |

Episodes: for each held-out stop event, three variants: genuine, non-compliance splice,
estimator-lies (non-compliance splice with sport velocity and yaw_speed set to 0 after the stop).
Quiet thresholds come from faultwalk1 only.

ASSUMPTION: the GO2 complied with each StopMove in the PHM walks (protocol "still" segments
followed; no record of non-compliance). There is no external ground truth such as motion capture.

## Confound checklist

- A1 onset: onsets start 10 s into each recording. Report frac of alarms in the first 1 s of each run (expected 0).
- A2 exposure: every arm sees identical episodes and an identical 2.0 s scoring window; FA/h uses identical clean hours.
- A3 seeds: units are episodes (about 39 onsets x class on 3 held-out detection recordings; 36 stop events on 2 recordings). Recording-level clustering is acknowledged: C1 additionally requires the difference to be positive in each of the 3 held-out recordings. CIs: bootstrap over 30 s blocks within recordings (10,000 resamples).
- A4 bundling: HELIX contains B-rate exactly; per-check ablation reported.
- A6 outcome: detection of an injected fault and correctness of a stop verdict are proxies for avoided harm. Venue is a technical report and README evidence, not a paper headline.
- A7 dose: the alarm-rate "dose" is held-out clean FA/h, printed per arm. Stop: false FAILED/UNVERIFIED rate on genuine stops printed per arm.
- A8 floor: B-rate is at floor on content faults by construction; stated above. B-range is the non-trivial comparator.
- A9 selection: all thresholds, the rotation, the window W = 2.0 s and the fault magnitudes are fixed in this document or on DEV. Held-out is scored once, after a code-freeze commit whose hash is recorded with the results. Any later change is labelled developmental.
- A10 leakage: calibration (rotation, thresholds, quiet levels) uses DEV recordings only; split is by recording.

## Analysis

Detection rate per arm and class with Wilson 95% CI. Primary: pooled (F3a, F4 0.2, F5, F6)
HELIX minus union detection difference, block-bootstrap 95% CI. Latency median and p95 for
detected episodes. FA/h with exact Poisson 95% upper bound. C2: rates with Wilson CI.
One primary comparison per claim; everything else is exploratory.

## Kill criteria (numeric; negatives will be published in the repo)

- K1: HELIX held-out clean FA/h > 2.0 (pooled detection held-out plus clean FA held-out) kills C1.
- K2: pooled content-fault difference CI lower bound <= 0.30, or difference not positive in all 3 held-out recordings, kills C1.
- K3: HELIX VERIFIED on genuine held-out stops < 0.80, or wrong-VERIFIED on estimator-lies > 0.20, kills C2.
- K4 (component decision): if B-rate is within 0.05 of B-z on F1 and F2 detection at no more FA, AnomalyCore is removed. If B-z beats B-rate by more than 0.10 at no more FA, it stays.
- Positive-control failure (F1, bias 1.0, non-compliance below 0.95) means the pipeline is broken and no result is reported until fixed.

Stopping rule: one held-out scoring pass per claim after code freeze.
