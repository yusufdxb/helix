# Measuring the HELIX anomaly-detector operating point

The detector threshold was raised from 3.0 to 4.0 and a `min_anomaly_duration_s`
gate was added after Session 8, both by feel. This directory replaces the guess
with a measurement taken from telemetry already on disk. No robot, no
simulator, no ROS runtime was used: everything here is pure Python over
recorded bags.

**Headline result: the shipped configuration is not conservative, it is blind.**
Over 901.5 s of real idle GO2 telemetry it raises zero false alarms, but on
synthetic sustained faults injected into that same real data it detects a
50 percent rate collapse 0 times out of 62 and a 10 sigma sustained rate step
1 time out of 62. The only failure it reliably catches is total topic death.
A different operating point buys back that detection at no measured cost in
false alarms.

---

## 1. What was measured, and on what

| Axis | Status | Source |
| --- | --- | --- |
| False alarms | **REAL** | GO2 standing idle, 901.5 s across three captures |
| Detection rate and latency | **SYNTHETIC onset on a REAL baseline** | fault models injected into the same idle streams after a full real warmup |

Nothing on the false-alarm axis is simulated. Nothing on the detection axis is
presented as a real fault observation.

### Real idle baselines (901.5 s total)

| Name | Kind | Duration | Metrics | Provenance |
| --- | --- | --- | --- | --- |
| `adapter_session` | RECORDED-METRICS | 134.5 s | 5 rate metrics (plus 8 flat ones) | `hardware_eval_20260406/bags/helix_adapter_session` recorded `/helix/metrics`, i.e. the detector's literal input |
| `extended_5min` | RECONSTRUCTED-RATE | 329.5 s | 5 rate metrics | `hardware_eval_20260406/bags/extended_5min` raw arrival timestamps for `/utlidar/imu`, `/utlidar/cloud`, `/utlidar/robot_pose`, `/gnss`, `/multiplestate`, replayed through `RateWindow` |
| `session8_sportmodestate` | RECONSTRUCTED-RATE | 439.0 s | 1 rate metric | Session 8's own bag, `/sportmodestate` arrival timestamps (130119 messages), replayed through `RateWindow` |

`hardware_eval_20260406/bags/` is **not** stripped: all five bag directories
still contain their `.db3` files. `metadata.yaml` files are absent, which is why
the extraction reads the sqlite tables directly rather than through `rosbag2`.

`helix_faults_session` is excluded from the false-alarm axis on purpose: it is
the fault-injection capture, so its faults are not false.

Reconstruction is faithful to `helix_adapter/rate_window.py` and
`topic_rate_monitor.py`: 5.0 s sliding window, `(n-1)/span`, NaN when empty,
0.0 when a single sample, published on a 0.5 s timer, metric named
`rate_hz/<topic with slashes stripped>`.

ASSUMPTION: rosbag2 stores the recorder process's receive timestamp, while
`RateWindow` uses the monitor node's callback-dispatch time. Both are
receiver-side arrival times on the same host over the same DDS transport, so
the reconstruction is a close proxy, not an identity. The two `RECORDED-METRICS`
captures need no such assumption and agree with the reconstructions on noise
scale to within about 20 percent.

### Is the harness actually the shipped detector?

Yes, and it is proven rather than asserted.
`analysis/detector_replay.py::load_shipped_process_sample()` parses
`src/helix_core/helix_core/anomaly_detector.py` with `ast`, lifts
`AnomalyDetector._process_sample` out of the class, and compiles it against
stdlib-only globals. The **shipped bytecode** then runs against a stub object,
with no `rclpy` and no node instantiation.
`tests/test_detector_sweep.py` runs the shipped method and the mirror side by
side over four parameter sets on both synthetic adversarial streams and real
recorded hardware telemetry, and requires identical fault output. 19 tests,
all passing:

```
$ python3 -m pytest tests/test_detector_sweep.py -q
...................                                                      [100%]
19 passed in 0.88s
```

The escalation model is validated against the real Session 8 outcome too. Feeding
the 30 recorded Session 8 faults through the R1/R2 rules reproduces exactly the
9 `STOP_AND_HOLD` actions and 5 `RESUME` actions that the live system published.

The source detector file was not modified.

---

## 2. What Session 8 actually was

From `~/workspace/helix-demo-build/events.json` (439.02 s, robot idle the whole
run, `cmd_vel_max_abs` 0.0):

- 30 ANOMALY faults = **246 faults per hour**
- 9 `STOP_AND_HOLD` commands = **73.8 stops per hour**, in 5 distinct stop
  episodes
- smallest fault `|z|` is 3.01, so that run used `zscore_threshold: 3.0`
- faults by metric: `rate_hz/multiplestate` 13, `rate_hz/gnss` 8,
  `rate_hz/utlidar_robot_odom` 5, `rate_hz/utlidar_imu` 4
- only the `rate_hz/utlidar*` faults escalate (rule R1), so `robot_odom` and
  `imu` alone caused all 9 stops

---

## 3. Results

### 3.1 The shipped configuration cannot see a sustained fault

`zscore_threshold 4.0, consecutive_trigger 3, window_size 60, min_anomaly_duration_s 2.0`

| Fault model (SYNTHETIC, 62 injections each) | Detected |
| --- | --- |
| topic death (NaN) | 62 / 62 |
| 50 percent rate collapse | **0 / 62** |
| 10 sigma sustained step | **1 / 62** |
| 5 sigma sustained step | 0 / 62 |
| 3 sigma sustained step | 0 / 62 |
| 5 sigma ramp over 30 s | 16 / 62 |

The mechanism is self-masking, and it is a direct consequence of two shipped
choices interacting. The z-score is evaluated against a rolling window that the
fault itself is filling. A sustained step therefore produces a score that spikes
and then decays as its own samples enter the baseline. With `window_size` 60 at
the 2 Hz metric cadence the score falls back below 4.0 at about t+2.0 s, which
is exactly when `min_anomaly_duration_s = 2.0` first permits an emission. The
gate opens after the evidence has gone. See
`results/helix_selfmasking.png`.

This also explains why `min_anomaly_duration_s` >= 5.0 detects **nothing**
except topic death, at any threshold, any window, any consecutive trigger: no
z-score against a self-contaminating window stays above threshold for 5 seconds.

Topic death is the exception because NaN is never appended to the window, so
the violation persists indefinitely. That is why the existing closed-loop
hardware validation passes while the detector is otherwise blind: the one
failure mode it is tested against is the one failure mode that cannot mask
itself.

### 3.2 Sweep

570 configurations: 3 statistics (population z-score, median / scaled MAD,
EWMA), thresholds 2.5 to 16, `consecutive_trigger` 1/3/5/8, `window_size`
20/30/60/120, `min_anomaly_duration_s` 0/2/5/10. Raw output in
`results/sweep.json`.

Selected rows, all at `consecutive_trigger 3`, `min_anomaly_duration_s 2.0`:

| statistic | thr | W | REAL faults / hr | REAL stops / hr | stale | 5 sigma | 10 sigma | 50 % collapse |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| zscore | 4.0 | 60 | 0.0 | 0.0 | 1.00 | 0.00 | 0.02 | 0.00 |
| zscore | 3.5 | 60 | 0.0 | 0.0 | 1.00 | 0.31 | 0.56 | 0.92 |
| zscore | 3.0 | 60 | 8.0 | 0.0 | 1.00 | 0.55 | 0.90 | 0.94 |
| zscore | 4.0 | 120 | 0.0 | 0.0 | 1.00 | 0.39 | 0.90 | 0.95 |
| **zscore** | **3.5** | **120** | **0.0** | **0.0** | **1.00** | **0.58** | **0.97** | **1.00** |
| zscore | 3.0 | 120 | 0.0 | 0.0 | 1.00 | 0.81 | 1.00 | 1.00 |
| zscore | 2.5 | 120 | 0.0 | 0.0 | 1.00 | 0.95 | 1.00 | 1.00 |
| mad | 6.0 | 60 | 0.0 | 0.0 | 1.00 | 0.56 | 0.92 | 1.00 |
| mad | 3.0 | 60 | 67.9 | 70.0 | 1.00 | 0.89 | 1.00 | 1.00 |
| ewma | 4.0 | 60 | 0.0 | 0.0 | 1.00 | 0.00 | 0.00 | 0.00 |

Statistic findings:

- **MAD is more sensitive but not safer.** Its median/MAD baseline resists
  contamination, so it self-masks far less and detects a 50 percent collapse at
  every threshold tested. But real idle rate data has a heavy tail that MAD
  scores very aggressively: the maximum MAD score on healthy idle data is 80.0
  and idle runs of 8 to 13 consecutive above-threshold samples occur at
  threshold 4.0. It needs a threshold near 6 to reach zero false alarms, and at
  that point it is no better than z-score at `window_size` 120.
- **EWMA is the worst of both.** Its own variance estimate is contaminated by
  the fault a sample at a time, so it self-masks like z-score, and its idle tail
  is the heaviest of the three (maximum idle score 403 at `window_size` 60).
- **Window size is the parameter that actually matters** and it was never swept
  before. Doubling `window_size` from 60 to 120 halves the rate at which a fault
  poisons its own baseline, which is what lets the 2.0 s duration gate open
  while the score is still above threshold.

### 3.3 Margin, not just counts

A zero count over 15 minutes is weak evidence: the 95 percent Poisson upper
bound on 0 events in 901.5 s is still 12.0 faults per hour. The stronger,
exposure-independent measurement is how close idle noise gets to firing
(`results/idle_margin.json`, 5999 scored idle samples):

| statistic | W | idle p99 | idle p99.9 | idle max | longest idle run above 3.0 | above 3.5 | above 4.0 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| zscore | 20 | 3.87 | 11.56 | 25.40 | 5 | 3 | 2 |
| zscore | 30 | 3.58 | 9.64 | 25.93 | 4 | 3 | 2 |
| zscore | 60 | 3.26 | 7.04 | 17.58 | 6 | 3 | 2 |
| zscore | 120 | 2.96 | 4.97 | 12.87 | **2** | **2** | **2** |
| zscore | 180 | 2.84 | 4.59 | 12.87 | 2 | 2 | 1 |
| mad | 60 | 4.45 | 20.33 | 80.01 | 13 | 12 | 11 |
| ewma | 60 | 3.18 | 15.57 | 403.65 | 5 | 3 | 2 |

Emission requires `consecutive_trigger` violations and then
`min_anomaly_duration_s` more seconds, which at 2 Hz means 5 consecutive
above-threshold samples for the recommended gate. At `window_size` 120 real idle
noise never exceeded 2 in a row at any threshold from 3.0 up. At `window_size`
60 the shipped threshold 4.0 leaves a margin of one single sample.

---

## 4. Recommended operating point

```yaml
helix_anomaly_detector:
  ros__parameters:
    zscore_threshold: 3.5        # was 4.0
    consecutive_trigger: 3       # unchanged
    window_size: 120             # was 60
    min_anomaly_duration_s: 2.0  # unchanged
```

Also written to `analysis/recommended_helix_params.yaml`. The shipped config
file was not edited.

**What it buys (SYNTHETIC, 62 injections per model, recommended vs shipped):**

| Fault model | Recommended | Shipped |
| --- | --- | --- |
| topic death | 62 / 62 | 62 / 62 |
| 50 percent rate collapse | **62 / 62** | 0 / 62 |
| 10 sigma sustained step | **60 / 62** | 1 / 62 |
| 5 sigma sustained step | **36 / 62** | 0 / 62 |
| 3 sigma sustained step | 21 / 62 | 0 / 62 |
| 5 sigma ramp over 30 s | 19 / 62 | 16 / 62 |

**What it costs:**

- **Detection latency: 2.5 s median, 2.5 s p90, 4.0 s worst case** for every
  step model. This is unchanged from the shipped config. It is not a property of
  the threshold: at a 0.5 s metric cadence, `consecutive_trigger 3` needs 1.5 s
  and `min_anomaly_duration_s 2.0` needs 2.0 s, so 2.5 s is the floor. Buying
  latency below 2.5 s means lowering those two gates, and that is what costs
  false alarms: `min_anomaly_duration_s 0.0` at `window_size` 60 and threshold
  3.0 gives 1.5 s latency and 39.9 false faults per hour.
- **False alarms: 0 measured**, in 901.5 s of real idle telemetry, identical to
  the shipped config. 95 percent Poisson upper bound 12.0 faults per hour;
  0 false `STOP_AND_HOLD` commands over 463.0 s of utlidar-bearing exposure,
  upper bound 23.3 stops per hour.
- **Slow drift is still mostly missed.** A 5 sigma degradation ramped over 30 s
  is detected 19 / 62 times. The best zero-false-alarm configuration in the
  entire sweep still only reaches 46 / 62. A rolling-window z-score is
  structurally blind to drift slower than its own window; no parameter choice
  fixes that, only a different detector (for example a fixed reference baseline
  captured at a known-good moment, or a CUSUM) would.

`zscore_threshold: 3.0` with `window_size: 120` is the more sensitive
alternative: same 0 measured false alarms, 5 sigma detection 50 / 62 instead of
36 / 62, but threshold 3.0 sits at the p99 of the idle score distribution
instead of roughly p99.5, so it has half the tail margin. Given the calibration
gap in section 5, 3.5 is the defensible choice.

---

## 5. Limitations, stated plainly

1. **These idle baselines are quieter than Session 8 was.** Replaying Session 8's
   own parameters (z > 3.0, k = 3, W = 60, duration gate off) over these
   baselines yields 39.9 faults per hour, while Session 8 itself measured 246
   per hour. The measured false-alarm rates here are a **lower bound**, roughly
   6x optimistic against that one reference point.
2. **`rate_hz/utlidar_robot_odom` is absent from every available idle capture.**
   It caused 5 of Session 8's 9 stops and is the noisiest metric in the fault
   record (relative window sigma 3.8e-3 to 6.0e-3, versus 1.2e-3 to 1.4e-3 for
   `utlidar_imu` here). The closest proxy on hand is
   `rate_hz/sportmodestate` at 5.2e-3, which is included. Until a bag with
   `/utlidar/robot_odom` exists, the false-alarm number for the metric that
   caused most of the real stops is unmeasured.
3. **Session 8's detector input was never recorded.** Only its outputs were. The
   30 Session 8 faults cannot be replayed sample by sample; they are used as
   ground truth for the escalation model and as a calibration reference, not as
   a replay source.
4. **15 minutes of idle exposure is not enough to certify a low false-alarm
   rate.** It is enough to rank configurations against each other and to reject
   the ones that fire repeatedly, which is what is done here.
5. **All detection numbers are synthetic.** They are honest about the detector's
   response to a defined signal injected into real noise, and they say nothing
   about how often those signals occur in the field.
6. The escalation model counts one `STOP_AND_HOLD` per qualifying fault, which
   is what `helix_diagnosis` does today and what the 9 Session 8 actions
   confirm. It does not model twist_mux latency or actuation.

---

## 6. Files

| File | What it is |
| --- | --- |
| `extract_telemetry.py` | pulls recorded `/helix/metrics` and reconstructs rate metrics from raw arrival timestamps into `data/` |
| `detector_replay.py` | the harness: shipped-code loader, mirrored detector with pluggable statistic, R1/R2 escalation model |
| `sweep.py` | 570-config sweep, real false alarms and synthetic latency, writes `results/sweep.json` |
| `idle_margin.py` | idle score distribution and longest above-threshold run, writes `results/idle_margin.json` |
| `plot.py` | the two figures |
| `recommended_helix_params.yaml` | the proposed config, not loaded by anything |
| `score_capture.py` | scores a bag against shipped and proposed settings and compares with live faults (section 7) |
| `gate_edge.py` | shows the live-vs-replay disagreement on the 2026-10-06 capture is the duration gate sitting on the 0.5 s sample grid (section 7) |
| `results/helix_operating_point.png` / `.svg` | main figure: tradeoff, per-fault-model detection, idle margin |
| `results/helix_selfmasking.png` / `.svg` | why the shipped config is blind |
| `../tests/test_detector_sweep.py` | shipped-vs-mirror equivalence, Session 8 escalation reproduction, and pins for every number quoted above |

Reproduce end to end:

```bash
python3 analysis/extract_telemetry.py
python3 analysis/sweep.py          # about 70 s
python3 analysis/idle_margin.py
python3 analysis/plot.py
python3 -m pytest tests/test_detector_sweep.py -q
```

---

## 7. Loaded standing capture, 2026-10-06 (18.4 min)

A real HELIX Tier-1 capture on a GO2 payload computer, robot standing still,
with a voice-interaction stack running on the same computer and a low-priority
compile from about 00:34Z (not separated here). Conditions, layout and side
evidence: [`../hardware_eval_20261006/README.md`](../hardware_eval_20261006/README.md).
Reproduce: `python3 analysis/score_capture.py` (writes
`results/loaded_standing_20261006.json` and `data/metrics_loaded_standing_20261006.json`).

The recorded `/helix/metrics` stream (1103.2 s, 13 metrics, 28691 samples, 0 NaN)
was replayed through the shipped detector code at both operating points.
`rate_hz/utlidar_robot_odom` is present in this capture, unlike the idle baselines.

| Operating point | False faults | Per hour | Without 1 s cooldown | STOP-producing faults | Longest above-threshold run, per rate metric (need 5) |
| --- | --- | --- | --- | --- | --- |
| Shipped (4.0 / 60) | 0 | 0.00 | 0 | 0 | gnss 5, multiplestate 3, utlidar_robot_pose 3, utlidar_cloud 1, utlidar_robot_odom 1 |
| Proposed (3.5 / 120) | 4 | 13.05 | 7 | 0 | gnss 8, utlidar_robot_pose 3, multiplestate 2, utlidar_robot_odom 2, utlidar_cloud 1 |

All 4 proposed-config faults are `rate_hz/gnss`. A STOP-producing fault is any
`rate_hz/utlidar*` fault; there were none under either config.

**The proposed operating point is NOT confirmed on this capture.** It raised
`rate_hz/gnss` false faults that the idle baselines (section 1) did not. The
shipped config also has no margin on gnss: its longest run is 5, exactly the
5 needed to emit.

### Resolved: live vs replay disagreement is a duration gate on the sample grid

The live shipped detector emitted 1 fault (`rate_hz/gnss`, z 4.26, t = 760.2 s
into the bag); replay of the same recorded metrics at the shipped parameters
emitted 0. `analysis/gate_edge.py` shows why:

- Both paths saw identical inputs. The live fault payload (value 0.9982, window
  mean 0.9993, window std 0.000248, z 4.26, 5 consecutive violations) matches
  the replayed sample at t = 760.198 s exactly.
- `min_anomaly_duration_s` is 2.0 s and the metric is published every 0.5 s, so
  the gate sits exactly on the sample grid: the fifth violating sample arrives
  4 x 0.5 = 2.0 s after the first. In the bag's receive stamps that interval is
  1.999984 s (16 us short), so replay's `elapsed >= 2.0` fails. The live node
  measures the same interval with its own `time.monotonic()` at callback time,
  where it came out at or above 2.0 s, so it emitted. Arrival jitter is
  0.39 ms (sd) over the capture, two orders of magnitude larger than that gap.
- Moving the gate off the grid by 1 ms (`min_anomaly_duration_s` 1.999) makes
  the shipped body emit the same fault at t = 760.2 s in replay.

Consequence: with the shipped parameters, emission needs 5 OR 6 consecutive
violations, decided by sub-millisecond timing. "5 required" in section 1 and
in `recommended_helix_params.yaml` is the lower of the two, so it is the
conservative figure for false-alarm margin, but neither live nor replay is
deterministic at that boundary, and the same applies to every replay result in
this directory whose streak ends exactly at the gate. Fix options, none
applied (each changes detection latency or behaviour): set the gate off the
grid (e.g. 1.75 s for 5 samples, 2.25 s for 6), gate on a sample count instead
of elapsed time, or compare with a half-period tolerance.

### Limits

- One 18.4 min capture. Zero events in that time still allows roughly 12 per hour
  at 95 % confidence, so "0 false faults" is not a rate of zero.
- Standing, not walking. Gait-induced rate and pose changes are not exercised.
- No injected faults, so detection was not measured here.
- A compile load window and a voice-interaction stack share the computer; the
  compile window is not separated.
- `/utlidar/imu` was excluded from topic_rate_monitor.
- The build was commit b31a9ce; Tier-1 equivalence to main is assumed.
