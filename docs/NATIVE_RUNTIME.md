# HELIX native runtime

`helix_native.launch.py` runs the monitoring, supervision and actuation path
as C++17 `rclcpp` processes. The Python packages remain the reference
implementation and are still used for offline analysis, tests and the
optional LLM explainer. No Python node runs on the robot in this profile.

This document covers what changed, why it changed, and exactly what has and
has not been shown on the GO2. The last section is the honest summary. Read
it before you quote any number.

## Pipeline

```
GO2 topics ──► helix_topic_rate_monitor ──► /helix/metrics ──► helix_anomaly_detector
               (GenericSubscription,                           (AnomalyCore)
                no deserialization)                              │ /helix/faults
/rosout ─────► helix_log_parser ────────► /helix/faults          │ /helix/fault_clear
                                                                  ▼
/helix/heartbeat (rate monitor, detector, log parser) ──► helix_supervisor
                                                          (one fault-state machine)
                                                                  │ /helix/hold
                                                                  ▼
upstream /cmd_vel sources ─────────────────────────────► helix_arbiter (C++)
                                                          sole velocity output
                                                                  │
                                                                  ▼
                                                          helix_go2_sport_sink (C++)
                                                          StopMove 1003 / Move 1008 only
                                                                  │ /api/sport/request
                                                                  ▼
                                                                GO2
/api/sport/response + /lf/sportmodestate ──► sink feedback ──► /helix/sink/status ──► supervisor
```

| Process | Package | Replaces (Python) |
|---|---|---|
| `helix_topic_rate_monitor` | `helix_sensing_cpp` | `helix_adapter.topic_rate_monitor` |
| `helix_anomaly_detector` | `helix_sensing_cpp` | `helix_core.anomaly_detector` |
| `helix_log_parser` | `helix_sensing_cpp` | `helix_core.log_parser` |
| `helix_supervisor` | `helix_supervisor_cpp` | `heartbeat_monitor`, `diagnosis_node`, `context_buffer`, `recovery_node` |
| `helix_arbiter` | `helix_arbiter_cpp` | `helix_arbiter.helix_arbiter` |
| `helix_go2_sport_sink` | `helix_go2_sink_cpp` | `helix_arbiter.go2_sport_sink` (kept as `sink_backend:=python`) |

The native profile does not run `json_state_parser` or
`pose_drift_monitor`. Their metrics are not stop-worthy, and neither has
been ported yet.

CUDA was considered and rejected. The workload is a few scalar metrics at
2 Hz plus message counting. A GPU context would cost more power and wake-up
latency than the whole computation.

## Weaknesses fixed

| Weakness (Python stack) | Native behavior | Where |
|---|---|---|
| R4: a CRASH mapped to `LOG_ONLY`, so a dead sensing node never stopped anything | A CRASH of a node in `critical_nodes` (`"*"` in the native launch) is stop-worthy | `supervisor_core.hpp` (R4) |
| RESUME fired 3 s after the last fault, with no evidence that the fault was gone | A fault holds until evidence clears it. ANOMALY: the detector's clear record (4 normal samples). CRASH: 1 s of continuous heartbeats. Critical LOG_PATTERN: latched until operator release. After that, `clear_hold_s` (2 s) in CLEARING, then AWAIT_RELEASE (operator `/helix/supervisor/release`) unless `auto_resume` is set | `supervisor_core.hpp`, `anomaly_core.cpp` |
| Diagnosis and recovery kept separate state and could disagree about whether a stop was accepted | One process, one state machine. `SupervisorStatus` is the only answer to "is HELIX holding" | `helix_supervisor_cpp` |
| The `enable` parameter was not synced with the safety envelope | `hold = would_hold && actuation_enabled`, applied on the next publish when the parameter flips. The same machine runs either way, so `would_hold` is always visible | `SupervisorCore::hold()` |
| A node missing at startup was invisible (its heartbeat registry never saw it) | `expected_nodes` are seeded at start. The supervisor holds in STARTING, and a node never seen within `startup_grace_s` (5 s) raises CRASH "never started" | `SupervisorCore::start()` |
| `twist_mux` did not guarantee a stop: it went silent on idle and lock and forwarded NaN | No `twist_mux`. The arbiter is the only velocity output. It outputs zero on hold, on a missing or stale (> 0.5 s) HELIX state, and when no input is live; it rejects non-finite input and drops that source's last command. The sink sends StopMove on zero, on bad input, and on its own 0.5 s deadman | `arbiter_core.hpp` P1 to P10, `sink_core.hpp` |
| "Stop" meant that a zero was published | A stop has explicit stages: HOLD_REQUESTED, arbiter ack (`ArbiterStatus` HELIX_HOLD echoing `hold_id`), controller ack (`/api/sport/response` code 0 for a StopMove sent after the hold), then STOP_CONFIRMED (SportModeState speed and yaw rate below threshold for 0.3 s). A timeout gives STOP_UNCONFIRMED and the hold stays asserted | `SupervisorStatus.msg`, `SinkStatus.msg` |
| Detector self-masking: a sustained fault cleared itself after a few seconds | Frozen baseline, std floor, clear hysteresis, rate anchor; see [`DETECTOR_EVAL.md`](DETECTOR_EVAL.md) | `anomaly_core.cpp` |
| A dead topic reported a near-normal rate for 5 s | `stale_after_s` 1.0: NaN one second after the last arrival | `rate_window.hpp` |

All supervisor, arbiter and sink times use the host steady clock
(`time.monotonic()` in Python, `steady_clock` in C++). Ack timestamps can be
compared across processes without wall-clock skew.

## Safety posture of the launch

The defaults are non-actuating: `sink_mode:=dry_run` creates no
`/api/sport/request` publisher, the arbiter runs on `/helix_dry/*`, and the
supervisor starts with `actuation_enabled:=false`. Arming takes three
explicit choices (`sink_mode:=armed`, a non-dry arbiter config,
`actuation_enabled:=true`), and motion still needs an upstream command
source. The sink can produce only 1003 (StopMove) and 1008 (Move).
`make_request` refuses every other api id, including 1001 (Damp). On
SIGINT or SIGTERM the C++ sink sends three StopMove requests before it
exits.

## Parity and tests

Each C++ core has a Python counterpart, and the parity tests replay one
input stream through both:

| Package | Parity | Other tests |
|---|---|---|
| `helix_sensing_cpp` | detector: 134 Python parity cases; log parser: 16 (`test_log_parser_parity.py`); new detector extensions default off (`tests/test_param_parity.py`) | colcon: 296 tests, 0 failures |
| `helix_supervisor_cpp` | none: it is new, and replaces 4 Python nodes rather than porting one | 49 tests |
| `helix_arbiter_cpp` | 233 parity cases | 30 process tests |
| `helix_go2_sink_cpp` | 37 (3 modes x 12 seeds + branch coverage) | 15 gtests; 79 tests in total without `unitree_api` (CI) |

These counts are from local runs on 2026-10-17 at commits up to e4f3032.
GitHub CI has not run on this branch: it has not been pushed.

## Evidence

### Python baseline on the GO2 (hardware, 2026-10-17)

`hardware_eval_20261017/pybaseline/`: the full Python closed-loop stack on
the Orin NX, 30 min (1796 s of samples), robot lying, sink in dry_run, the
same rate-monitor topic set as the native profile (including the 250 Hz
`/utlidar/imu`). Per-PID mean, where 100 % is one core:

| Process | CPU | RSS |
|---|---|---|
| topic_rate_monitor | 36.55 % | 62.4 MB |
| arbiter | 7.61 % | 55.5 MB |
| heartbeat_monitor | 7.04 % | 38.2 MB |
| go2_sport_sink | 5.85 % | 55.3 MB |
| context_buffer | 3.79 % | 38.4 MB |
| pose_drift_monitor | 3.66 % | 55.4 MB |
| recovery_node | 3.64 % | 54.2 MB |
| anomaly_detector | 3.43 % | 38.3 MB |
| json_state_parser | 2.36 % | 39.6 MB |
| diagnosis_node | 2.30 % | 40.0 MB |
| log_parser | 1.98 % | 38.3 MB |
| llm_explainer | 0.88 % | 47.5 MB |
| `ros2 run` wrapper | 0.00 % | 24.2 MB |
| **Total** | **79.08 %** | **587.4 MB** |

The first 10 minutes alone average 79.1 %, so a 10-minute native run is a
fair comparison.

### Native stack on the GO2 (hardware, 2026-10-17, non-actuating smoke)

Build 6ace4d7 on the Orin, `helix_native` defaults (dry_run, actuation off):
all 5 lifecycle nodes `active`, supervisor NOMINAL with no missing nodes,
sink `feedback_available` from the real `/lf/sportmodestate`. Live rates:
imu 247.8 Hz, robot_odom 150.6, robot_pose 18.75, cloud 15.39, gnss 1.0,
multiplestate 1.0. The robot was lying (mode 0, body height 0.073 m).

The first 10-minute native measurement run (20:31Z) was lost when the Orin
dropped off the network.

### Native stack cost on the GO2 (hardware, 2026-10-17, 10 min)

`hardware_eval_20261017/native/`: `scripts/hw/native_run.sh <build> 600 native`.
Conditions:
- Build 1254bbe, 22:40:37Z to 22:51:17Z, 595 s of samples (119 per process).
- The robot was lying, as in the Python baseline.
- `helix_native` defaults: sink in dry_run, actuation off.
- Same sampler, and the same per-PID mean, as the Python table.

| Process | CPU | RSS |
|---|---|---|
| helix_topic_rate_monitor | 2.69 % | 19.7 MB |
| helix_supervisor | 1.44 % | 14.3 MB |
| helix_go2_sport_sink | 1.12 % | 14.2 MB |
| helix_arbiter | 1.03 % | 14.6 MB |
| helix_anomaly_detector | 0.58 % | 15.1 MB |
| helix_log_parser | 0.49 % | 13.5 MB |
| **Total (6 processes)** | **7.35 %** | **91.5 MB** |

Against the Python baseline (79.08 % and 587.4 MB, 13 processes), that is
90.7 % less CPU and 84.4 % less RSS. Python's own first 600 s average
79.12 % CPU and 578.1 MB. The bag has 0 messages on `/helix/faults`, and
the supervisor reported NOMINAL in all 1200 status messages.

The two profiles do not do the same work. The native profile has no LLM
explainer, context buffer, pose-drift monitor or JSON state parser, and its
supervisor replaces the heartbeat, diagnosis and recovery nodes. So this is
the cost of the profile that ships, not a node-for-node port.

### Native stack cost, first attempt (175 s, robot standing, partial)

`hardware_eval_20261017/native_partial/`, recorded with
`scripts/hw/native_run.sh`:
- Build 1254bbe, `helix_native` defaults: sink in dry_run, actuation off.
- The robot was standing, not lying as in the Python baseline.
- The run was planned for 600 s. I stopped it after 175 s of samples (35 per
  process) to free the robot for the motion trials.
- Same sampler and the same per-PID mean as the Python table.

| Process | CPU | RSS |
|---|---|---|
| helix_topic_rate_monitor | 2.63 % | 17.3 MB |
| helix_supervisor | 1.42 % | 15.8 MB |
| helix_go2_sport_sink | 1.07 % | 11.9 MB |
| helix_arbiter | 0.94 % | 12.5 MB |
| helix_anomaly_detector | 0.55 % | 13.0 MB |
| helix_log_parser | 0.46 % | 12.9 MB |
| **Total (6 processes)** | **7.06 %** | **83.5 MB** |

This first attempt was stopped early, and it agrees with the 10-minute run.

### Stop chain against a simulated GO2 (rehearsal, not hardware)

`scripts/hw/native_stop_test.py` against `helix_fake_go2` (the Python
dynamics stand-in) on a loopback DDS domain, with the native stack armed:
C++ supervisor, arbiter and sink, `arbiter_hw.yaml`. The test injects an
ANOMALY on `rate_hz/utlidar_helix_injected` while the fake robot moves at
vx 0.15.

| Stage | Verdict | Hold requested | Arbiter ack | Controller ack | Measured stop |
|---|---|---|---|---|---|
| ack | PASS | 0.08 ms | 0.32 ms | 442 ms | (confirmed at 453 ms) |
| motion | PASS (24/24 checks) | 0.06 ms | 0.21 ms | 0.41 ms | 167 ms (SportModeState), odometry still after the deadline |

The motion stage also checks that no Move is sent while held, that an
explicit clear leads to AWAIT_RELEASE (no auto resume), that the operator
release is accepted, and that the robot stays still for 3 s after release.

In the ack stage the robot was already still. The controller ack had to
wait for the next 2 Hz StopMove repeat (442 ms). That stage found a real
bug: the sink's ack stamp kept the first acknowledged StopMove, so a hold
asserted later never saw an ack newer than itself. The stamp now follows
the newest acknowledged StopMove in both the Python and C++ cores (commit
3a88313, with tests).

The fake GO2 answers every StopMove with code 0, and it supplies both
SportModeState and odometry. This rehearsal therefore proves the HELIX
side of the chain. It says nothing about how the real controller acks or
how long the real robot takes to stop.

### Stop chain on the real GO2 (hardware, 2026-10-17, armed)

`hardware_eval_20261017/native_stop/`: build 1254bbe plus the
`helix_bringup` dependency fix (ddf816c), run with
`scripts/hw/native_stop_hw.sh`. The C++ sink was armed and was the only
HELIX publisher on `/api/sport/request`. The supervisor had actuation
enabled. The robot was standing in `mcf` (body height 0.32 m), with the
operator present on the remote and the area clear. Every run is listed,
including the failures:

| Run | Stage, args | Verdict | Note |
|---|---|---|---|
| 221913 | ack | FAIL (1/3, preflight) | deployment bug: `--packages-up-to helix_bringup` did not build `helix_supervisor_cpp`, so the launch failed. Fixed in ddf816c, which adds a launch test |
| 222125 | ack | **PASS** (18/18) | robot still |
| 222205 | motion, vx 0.15, settle 1.0 s | FAIL (23/24) | `moving at injection` 0.032 m/s: the stock gait lurched to 0.17 m/s, then stalled at 0.03 to 0.05 m/s while Move kept arriving |
| 222336 | motion, vx 0.3 | refused | the runner rejects vx above the sink's 0.25 m/s limit. No Move was sent; the armed sink sent only its periodic StopMove (32, all code 0) |
| 222412 | motion, vx 0.25, settle 0.3 s, abort 0.40 | **PASS** (24/24) | robot moving at 0.130 m/s at injection |
| 223203 | same as 222412 | **PASS** (24/24) | robot moving at 0.316 m/s at injection |
| 223245 | same as 222412 | FAIL (9/10), overspeed abort | SportModeState reported 0.43 m/s 25 ms after the first Move, while odometry read 0.015 m/s. The runner stopped sourcing, the sink sent StopMove 1 ms later, and the robot stayed standing. No fault was injected |

Between 222412 and 223203 the robot was found lying (body height 0.072 m). The native stack was in dry_run then, with no `/api/sport/request` publisher, and no other ROS process was running. It was presumably laid down from the remote or app. The operator stood it with the remote before 223203. Trials stopped after 223245: the abort rule is unchanged, and changing it is not a run-time decision.

Supervisor-reported latencies, measured from the injected fault:

| Run | Hold requested | Arbiter ack | Controller ack (StopMove, code 0) | STOP_CONFIRMED |
|---|---|---|---|---|
| 222125 ack | 0.15 ms | 0.62 ms | 159 ms | 206 ms |
| 222412 motion | 0.18 ms | 0.65 ms | 2.6 ms | 926 ms |
| 223203 motion | 0.17 ms | 0.64 ms | 1.7 ms | 575 ms |

The runner's own motion measurements:

| Run | Speed at fault | SportModeState still after | Odometry max after the 1.5 s deadline | Peak speed | Travel, first Move to fault | Travel, fault to +3 s (odometry) |
|---|---|---|---|---|---|---|
| 222412 | 0.130 m/s | 636 ms | 0.024 m/s | 0.36 m/s | 0.057 m in 0.35 s | 0.051 m |
| 223203 | 0.316 m/s | 265 ms | 0.022 m/s | 0.32 m/s | 0.060 m in 0.34 s | 0.021 m |

`scripts/hw/stop_bag_check.py` re-reads each bag without the runner's
recorder. It confirms the following:

| Run | Move before / after the fault | First request after the fault | Response codes |
|---|---|---|---|
| 222125 ack | 0 / 0 | StopMove at 158 ms, code 0 at 159 ms | 46 StopMove code 0, 1 unanswered (the first, at sink startup) |
| 222205 motion | 18 / 0 | StopMove at 0.55 ms, code 0 at 1.8 ms | 52 StopMove and 18 Move code 0, 1 unanswered (the first, at sink startup) |
| 222412 motion | 6 / 0 | StopMove at 0.73 ms, code 0 at 2.4 ms | 53 StopMove and 6 Move code 0 |
| 223203 motion | 6 / 0 | StopMove at 0.66 ms, code 0 at 1.5 ms | 53 StopMove and 6 Move code 0 |

The nav source kept streaming vx during the hold, and no Move reached the
robot. After the explicit clear and operator release, the robot stayed still
for 3 s and no Move was sent (every passing stage).

What these runs do not show:
- **A start transient, not steady walking.** The fault landed about 0.35 s
  into the motion. At 0.25 m/s or below, the stock gait does not keep a
  steady walk: run 222205 stalled. So the stop times (265 and 636 ms) come
  from a decelerating start, not from cruise speed. Run 222205 is the
  closest thing to a counterfactual: Move kept arriving there and the robot
  crept at 0.03 to 0.05 m/s. No run has measured how far the robot would
  have travelled without the hold.
- **Few trials.** 2 passing motion trials out of 4 motion attempts at the
  shipped limits, plus 1 ack pass. No repeat statistics.
- **The overspeed abort trips on one SportModeState sample.** Run 223245
  aborted on a 0.43 m/s reading that odometry did not confirm. The abort
  failed safe: it stopped commanding. But at vx 0.25 the abort interrupts
  1 trial in 3 before the fault is injected.
- **Idle-robot ack waits for the repeat.** When the robot is already still,
  the hold does not trigger an immediate StopMove. The sink's next 2 Hz
  repeat carries it, so the controller ack took 159 ms. From motion it was
  immediate (0.73 ms).
- **The fault was injected.** The fault was one synthetic `FaultEvent` on
  `rate_hz/utlidar_helix_injected`. No real sensor degraded.

## Proven, partial, unverified

| Claim | Status | Evidence |
|---|---|---|
| C++ cores make the same decisions as the Python references | **Proven (software)** | parity suites above |
| A native stack runs on the Orin against the live GO2 graph (non-actuating) | **Proven (hardware, short smoke)** | smoke result above |
| The sink reads real GO2 feedback (`/lf/sportmodestate`) | **Proven (hardware)** | smoke: `feedback_available` true |
| Python stack cost: 79.1 % of one core, 587 MB | **Proven (hardware)** | `hardware_eval_20261017/pybaseline/procs.jsonl` |
| Native stack cost: 7.35 % of one core and 91.5 MB, against Python's 79.08 % and 587.4 MB (-90.7 % CPU, -84.4 % RSS) | **Proven (hardware, 10 min)** | `native/procs.jsonl`; robot lying, as in the baseline. The profiles do different work (see above) |
| Fault-state machine: R4, explicit clearance, a single state, enable sync, startup visibility | **Proven (software)** | 49 supervisor tests; rehearsal |
| Hold to arbiter zero to StopMove to ack to measured stop, end to end | **Proven (hardware, small n)** | `native_stop/`: ack run 222125; motion runs 222412 and 223203 |
| The GO2 acks a repeated StopMove from this sink with code 0 | **Proven (hardware)** | 277 StopMove answered with code 0 across all 6 armed bags; 2 without a recorded response, both the first request at sink startup (cause not established: possibly sent before the recorder matched `/api/sport/response`) |
| The GO2 physically stops on a HELIX fault | **Partial (hardware)** | 2 of 2 faults injected while the robot moved above 0.05 m/s stopped it from a start transient (0.13 and 0.32 m/s at the fault; SportModeState still at 636 and 265 ms; odometry confirms). Not from steady walking |
| No Move reaches the robot while held, with upstream streaming | **Proven (hardware)** | bag check: 0 Move after the fault in all 3 motion runs that injected one |
| Native detector config: 0 stop false alarms, 0 false clears, all 6 models detected | **Proven on held-out (short)** | [`DETECTOR_EVAL.md`](DETECTOR_EVAL.md): DEV of 1.057 h, plus a held-out pass on 0.216 h (95 % bound about 13.9 false alarms per hour); synthetic onsets on real streams |
| Live fault injection (topic loss, rate collapse, drift) on the robot | **Unverified** | not run |
| Crash detection by SIGKILL of a live node on the Orin | **Unverified** | supervisor tests only |
| DDS liveliness compared with the heartbeat | **Unverified** | not evaluated |
| Long-run stability and timer jitter of the native stack | **Unverified** | not run |

## Running it

Non-actuating, on the Orin:

```bash
scripts/hw/native_run.sh <build dir> 600 native     # 10 min, CPU/RSS + bag
```

Hardware stop test. It needs the robot standing, an operator holding the
remote, a clear area and a verified e-stop. Stage `ack` sends StopMove only.
Stage `motion` defaults to vx 0.15 m/s and aborts above 0.35 m/s. On the
GO2 that speed stalls the gait, so the passing run used the arguments below.
The runner refuses vx above the sink's 0.25 m/s limit.

```bash
scripts/hw/native_stop_hw.sh <build dir> ack
scripts/hw/native_stop_hw.sh <build dir> motion --vx 0.25 --settle-s 0.3 --abort-speed 0.40
python3 scripts/hw/stop_bag_check.py ~/helix_hw/<run>/bag
```

The wrapper launches the armed native stack, records the bag, runs one
stage and shuts the stack down. Each run writes `evidence/evidence.json`
with every check and the full event log.
