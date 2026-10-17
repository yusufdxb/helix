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

**Not measured:** native CPU and RSS. The 10-minute native measurement run
started at about 20:31Z, and the Orin dropped off the network at about the
same time (LabLink RED, TCP/22 timeout). The result has not been recovered.

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

## Proven, partial, unverified

| Claim | Status | Evidence |
|---|---|---|
| C++ cores make the same decisions as the Python references | **Proven (software)** | parity suites above |
| A native stack runs on the Orin against the live GO2 graph (non-actuating) | **Proven (hardware, short smoke)** | smoke result above |
| The sink reads real GO2 feedback (`/lf/sportmodestate`) | **Proven (hardware)** | smoke: `feedback_available` true |
| Python stack cost: 79.1 % of one core, 587 MB | **Proven (hardware)** | `hardware_eval_20261017/pybaseline/procs.jsonl` |
| Native stack CPU and RSS reduction | **Unverified** | run lost when the Orin went offline |
| Fault-state machine: R4, explicit clearance, a single state, enable sync, startup visibility | **Proven (software)** | 49 supervisor tests; rehearsal |
| Hold to arbiter zero to StopMove to ack to measured stop, end to end | **Partial: rehearsal only** | fake GO2; real robot not run |
| The GO2 acks a repeated StopMove from this sink with code 0 | **Unverified** | riskiest open assumption |
| The GO2 physically stops on a HELIX fault | **Unverified** | `native_stop_test.py --stage motion` not run on the robot |
| Native detector config: 0 stop false alarms, 0 false clears, all 6 models detected | **Partial: DEV only** | [`DETECTOR_EVAL.md`](DETECTOR_EVAL.md); no held-out score |
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
remote, a clear area and a verified e-stop. Stage `ack` sends StopMove only;
stage `motion` moves at vx 0.15 m/s and aborts above 0.35 m/s.

```bash
ros2 launch helix_bringup helix_native.launch.py sink_mode:=armed actuation_enabled:=true \
  arbiter_config:=$(ros2 pkg prefix helix_arbiter)/share/helix_arbiter/config/arbiter_hw.yaml \
  cmd_vel_out:=/helix_hw/cmd_vel
python3 scripts/hw/native_stop_test.py --stage ack --out <dir>
python3 scripts/hw/native_stop_test.py --stage motion --out <dir>
```

Each run writes `evidence.json` with every check and the full event log.
