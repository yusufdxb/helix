# Evidence audit (updated 2026-10-18)

Every HELIX result from the native-runtime campaign, sorted by what
produced it. A claim may only be stated at the class of its weakest link: a
hardware stop with a synthetic trigger is not a hardware detection, and a
simulated crash recovery is not a robot result.

## Evidence classes

| Class | Meaning |
|---|---|
| **HW-real** | Real robot, real trigger, real robot response |
| **HW-command** | Real robot and real controller, synthetic trigger (an injected `FaultEvent`) |
| **HW-passive** | Real robot graph, HELIX non-actuating; any read-only metadata queries explicitly listed |
| **Replay** | Recorded real streams replayed offline, with synthetic fault onsets |
| **Sim** | Fake GO2 (`helix_fake_go2`, a first-order stand-in) on localhost |
| **Bench** | Host benchmark under a synthetic load, no robot |
| **Unit** | Software tests and parity suites |

## Current controller investigation and readiness

The follow-up from `4ea69c8` adds no software watchdog. Physical motion remains
disabled, controller modes were not changed, and all earlier evidence files are
preserved. The PR remains draft. [Controller inspection](CONTROLLER_INSPECTION.md)
separates vendor interface evidence from measured controller behavior; the
[readiness record](../results/controller_readiness.json) explicitly blocks a
physical demonstration.

Read-only access and discovery could not identify/reach the payload. No remote
command succeeded. Current robot firmware, mode, command-silence behavior and
current-candidate ARM resource usage therefore remain unverified. API versions,
raw `LowState.version` fields and payload OS versions are not firmware identity.
The vendor's GO2-W one-second and B2W half-second comments do not establish a
timeout for the installed legged GO2.

The finite query-only inspector was tested with synthetic replies. A separate
20-second [host simulation](../results/nonactuating_native_host_sim.json) ran all
six default native processes with FakeGo2 and synthetic LowState: no Sport requests,
retained protective latch and fresh native status. Aggregate CPU averaged 2.33%
of one host core (sample maximum 4.99%); peak summed RSS was 93,740 KiB. These are
short x86 simulation measurements, not ARM, thermal or timing guarantees.

The [walking-stop protocol](WALKING_STOP_EXPERIMENT.md) requires independent body
tracking and foot-contact evidence, clock/calibration bounds, supported separate
operator-stop validation and physical containment covering final-command-path
failure. Its offline analyzer is tested only with synthetic inputs. The optional
`--keep-held` runner mode leaves the injected fault and protective latch active;
the prepared trial never clears or releases automatically. No physical trial or
handheld emergency action was performed. The manual's damping mapping establishes
an interface, not a measured independent stop guarantee.

## Stops: hardware commands, not hardware detections

Every stop the native stack sent to the real GO2 was triggered by one
synthetic `FaultEvent` (`rate_hz/utlidar_helix_injected`) published by the
test runner. No physical fault, degraded sensor or crashed node caused a
stop on the robot.

| Run | Class | What it shows | What it does not show |
|---|---|---|---|
| 221913 ack | (failed preflight) | deployment bug: supervisor package not built; fixed in ddf816c with a launch test | anything about stopping |
| 222125 ack | HW-command | hold 0.15 ms, arbiter ack 0.62 ms, StopMove answered code 0 at 159 ms (2 Hz repeat), STOP_CONFIRMED 206 ms | motion: the robot was already still |
| 222205 motion | HW-command (stage FAIL) | 18 Move before the fault, 0 after; StopMove 0.55 ms after the fault, code 0 at 1.8 ms | a moving robot: 0.032 m/s at the fault, the gait stalled |
| 222336 motion | none (refused) | the runner rejects vx above the sink's 0.25 m/s limit; no Move was sent | anything about stopping |
| 222412 motion | HW-command | 6 Move before, 0 after; StopMove 0.73 ms, code 0 at 2.4 ms; estimate 0.13 m/s at the fault, still at 636 ms | a walking robot (below) |
| 223203 motion | HW-command | 6 Move before, 0 after; StopMove 0.66 ms, code 0 at 1.5 ms; estimate 0.32 m/s at the fault, still at 265 ms | a walking robot (below) |
| 223245 motion | runner abort, no HELIX fault | the runner's abort fail-safe: it stopped sourcing on a one-sample estimator spike, and the arbiter and sink turned the silence into StopMove 1 ms later | a HELIX stop: no fault was injected |

Across the 6 recorded armed bags the controller answered 277 StopMove
requests with code 0. Two requests have no recorded response; both are the
first request at sink startup.

What the hardware class supports: **the HELIX command path, from a fault to
an acknowledged StopMove on the real controller with no Move while held,
works with sub-3 ms latency (n = 3 stops, 2 from motion).** What it does
not support: that HELIX stops a walking GO2.

### Why "walking" is not shown

- **Displacement.** The robot's state estimate reports 6 to 8 cm of body
  displacement per motion run. In 222205 it peaks at 7.6 cm and ends at
  1.4 cm, in 222412 it peaks at 6.2 cm and ends at 0.6 cm (4 s after the
  first Move). 223203 peaks at 8.4 cm and keeps 8.0 cm. A body shift or
  lean in stance fits the returning runs better than steps.
- **No step evidence either way.** `foot_force` and `foot_position_body`
  read zero in `mcf`. `SportModeState.mode` stays 0 even while the robot is
  driven at 1.4 m/s, so it does not indicate locomotion. `/lowstate` was not
  recorded during the trials; the stop-test wrapper records it now.
- **No second sensor.** `/sportmodestate` and `/utlidar/robot_odom` carry
  the same estimate: median velocity difference exactly 0, identical
  positions, odometry being every other SportModeState sample
  (`tests/test_stop_guard.py::test_the_two_topics_are_one_estimate`). The
  earlier docs called odometry an independent confirmation; that was wrong.
- **Onset spikes.** In 2 of 4 motion runs the peak speed is a sample
  within 26 ms of the first Move (0.36 m/s at 12 ms in 222412, 0.43 m/s at
  26 ms in 223245), faster than the body can accelerate. The "0.43 m/s
  against 0.015 m/s odometry" contradiction of run 223245 was a comparison
  of misaligned samples: the runner compared the spike with the previous
  odometry sample. The same 0.431 m/s value is on both topics, 0.2 ms
  apart.

### The 0.25 m/s limit and the gait

The 0.25 m/s sink limit was not raised. Untested hypotheses for why the
stock `mcf` gait does not engage: Move streams too short (0.35 to 1.0 s
before the fault), vx at or below 0.25 m/s under the gait's engagement
threshold (earlier lab notes: a clean trot needs vx of 0.5 or more), or no
BalanceStand before the first Move. Each needs motion on the robot and an
operator decision. No controller mode was changed for this audit.

### Posture change between runs

Between 222412 and 223203 the robot was found lying. The dry_run bag that
covers the window (`hardware_eval_20261017/native_partial/`, 22:26:16 to
22:29:13Z, `/helix/sink/status`) shows:

- HELIX sent nothing: sink `dry_run`, `last_api_id` 0 throughout.
- Still until 22:28:14Z (median 0.007 m/s).
- 19 s of motion, peaking at 1.42 m/s and 2.78 rad/s yaw.
- 0 m/s (median) from 22:28:33Z.

`/api/sport/request` and `/wirelesscontroller` were not recorded in that
window, so the source is not established (remote, app or another process on
the robot network).

### Controller-state ownership

- **Requests:** in all 6 armed bags the `/api/sport/request` ids form one
  consecutive sequence (the sink's), and every `/api/sport/response`
  answers one of those ids (`scripts/hw/stop_bag_check.py`: 0 sequence
  breaks, 0 unknown ids). No other process sent sport requests during the
  trials.
- **Publisher counts** on `/api/sport/request`, `/lowcmd` and
  `/wirelesscontroller` cannot show ownership on the GO2: the graph is full
  of unnamed bare-DDS entities.
- **Remote:** `/wirelesscontroller` publishes only while a stick or key is
  in use. It was not recorded during the trials. The runner now subscribes
  to it and fails a stage on any remote message (`no_remote_input`).

## Noisy-signal handling

Two separate consumers read the robot's speed:

1. **The HELIX stop path** (sink and supervisor) decides "still" from
   `/lf/sportmodestate` (20 Hz): speed at most 0.05 m/s and yaw at most
   0.10 rad/s held for 0.3 s. A spike resets the window, so noise can only
   delay `motion_stopped`, never fake it. The supervisor revokes
   `motion_stopped` if motion resumes. This path did not change.
2. **The test runner's own abort**, which governs only the runner's motion
   source. It was a single-sample threshold and aborted run 223245 on a
   spike. It is replaced by `scripts/hw/stop_guard.py`:
   - a sample above 0.60 m/s aborts at once;
   - a speed above the abort limit must persist for 30 ms (about 9 samples
     at 300 Hz);
   - more than 0.60 m of travel aborts;
   - a state topic silent for 0.2 s aborts;
   - a non-finite value aborts.

   Tested against the four recorded runs (19 tests): it ignores the 223245
   spike, stays quiet on the passing runs and trips on injected sustained
   overspeed within 30 ms plus one sample. At 0.60 m/s, 30 ms is under
   2 cm of travel.

Independent emergency stopping is unchanged: the remote, the e-stop and the
operator remain the stop of record. HELIX is never a substitute for them.

## Detector

| Result | Class |
|---|---|
| Native detector config selected on 1.057 h of real captures | Replay (synthetic onsets) |
| Held-out pass, 0.216 h: 0 stop false alarms (95 % bound about 13.9 per hour), 644/644 onsets, 0 false clears | Replay (synthetic onsets) |
| Earlier sessions: live ANOMALY and log-pattern faults on the GO2 (Python stack, log-only) | HW-passive ([`GO2_HARDWARE_EVIDENCE.md`](GO2_HARDWARE_EVIDENCE.md)) |
| Physical LiDAR cover and USB unplug produced no fault (Session 5) | HW-real, negative |

No physical sensor fault has been scored against the native configuration.

## Cost

### Same load, one-to-one components (Bench)

`scripts/bench_native_vs_python.py`, result `results/native_vs_python_bench.json`.

**Setup**
- Host: a 24-thread x86 desktop, not the Orin. The host had other work
  running (load average about 1 to 3.5).
- ROS_LOCALHOST_ONLY, own domain, CycloneDDS.
- 3 runs per arm of 120 s after a 20 s warmup, in the order python, cpp,
  cpp, python, python, cpp.

**Load** (one driver process, identical for both arms)
- GO2 topic rates: imu 250 Hz, odometry 150 Hz, pose 18.75 Hz, cloud
  15.4 Hz with a 64 KB synthetic payload, gnss and multiplestate 1 Hz.
- `/rosout` at 10 Hz, with one rule-matching ERROR line every 10 s.
- The nav command at 20 Hz.
- `/lf/sportmodestate` at 20 Hz.
- `/helix/hold` at 20 Hz (hold=false).

**Same work in both arms.** Every run produced 1680 `/helix/metrics`
messages, 14 faults and about 7010 arbiter outputs, of which all but 3 to 4
carried the nav command.

CPU is the percentage of one core: the mean over the runs of each run's mean.

| Component | Python CPU | C++ CPU | Change | Python RSS | C++ RSS | Change |
|---|---|---|---|---|---|---|
| topic_rate_monitor | 8.64 % | 0.82 % | -90.6 % | 54.2 MB | 17.3 MB | -68.1 % |
| anomaly_detector | 0.47 % | 0.06 % | -86.9 % | 49.0 MB | 14.9 MB | -69.6 % |
| log_parser | 0.55 % | 0.10 % | -82.3 % | 49.5 MB | 14.7 MB | -70.4 % |
| arbiter | 2.27 % | 0.31 % | -86.5 % | 49.6 MB | 14.7 MB | -70.4 % |
| go2_sport_sink (dry_run) | 2.50 % | 0.26 % | -89.7 % | 51.9 MB | 14.3 MB | -72.5 % |
| **Five components** | **14.42 %** | **1.54 %** | **-89.3 %** | **254.1 MB** | **75.8 MB** | **-70.2 %** |

Per-run CPU spread within an arm was at most 0.6 points (rate monitor,
Python: 8.35, 8.94, 8.63 %). The supervisor has no Python counterpart (it
replaces four nodes), so it is not in this table. The C++ detector runs the
native configuration (`DETECTOR_OVERRIDES`) and the Python one the shipped
configuration; their outputs matched in count on this load.

### On the Orin (HW-passive, cross-run)

From the two hardware cost runs, both with the robot lying:
- the Python baseline, 30 min at 19:27Z;
- the native run, 10 min at 22:40Z.

These are different runs with different node sets. The table shows each
component's mean over its process ids, after a 30 s warmup.

| Component | Python (Orin) | C++ (Orin) |
|---|---|---|
| topic_rate_monitor | 36.51 %, 62.5 MB | 2.69 %, 19.9 MB |
| arbiter | 7.59 %, 79.7 MB | 1.03 %, 14.7 MB |
| go2_sport_sink | 5.83 %, 55.3 MB | 1.11 %, 14.2 MB |
| anomaly_detector | 3.41 %, 38.3 MB | 0.58 %, 15.2 MB |
| log_parser | 1.97 %, 38.4 MB | 0.49 %, 13.5 MB |
| supervisor (replaces heartbeat, context, diagnosis, recovery: 16.70 %, 171.1 MB) | | 1.44 %, 14.4 MB |
| **Whole profile** | **79.08 %, 587.4 MB** | **7.35 %, 91.5 MB** |

The whole-profile row is the recorded run summary. The Python profile also
runs the LLM explainer, the JSON parser and the pose-drift monitor, which
the native profile does not. Ten minutes is not a soak.

## Crash recovery

`scripts/sim_native_crash_recovery.py`, result
`results/native_crash_recovery_sim.json`. **Class: Sim.**

The original 2026-10-17 run used a fresh armed native stack (actuation on, `auto_resume`
false) against the fake GO2, streams vx 0.2 m/s and waits until the sink is
sending Move and the supervisor is NOMINAL. It then SIGKILLs one process
while the nav source keeps streaming.

| Killed | What stopped motion | Last Move after the kill | First StopMove after the kill | Move after that StopMove | Recovery |
|---|---|---|---|---|---|
| anomaly detector | supervisor: missed heartbeat, CRASH fault, hold | 254 ms | 265 ms | 0 | **PASS**: the restarted detector led to CLEARING and then AWAIT_RELEASE, with no Move while waiting; the operator release returned NOMINAL and Move resumed |
| supervisor | arbiter: hold stream stale (`hold_timeout_sec` 0.5 s), forced zero | 435 ms | 495 ms | 0 | historical P0, fixed by the native arbiter latch below |
| arbiter | sink: input deadman (`input_timeout_sec` 0.25 s) | none | 237 ms | 0 | not tested |
| sink | nothing | none | none | none | **gap**: no process is left to send StopMove |

Findings:

- **Historical P0: supervisor restart dropped the latched hold.** In the original run:
  - The restarted supervisor started a new epoch.
  - It published hold=true, then hold=false 0.1 s later, as soon as every
    expected node was beating.
  - Move resumed 1.15 s after the restart, 198 Move requests in 12 s,
    with no operator release and `auto_resume` false.

  The native arbiter-owned protective latch now prevents that restart from
  releasing motion. The original result above is retained as the failure
  evidence, rather than reclassified as a passing run.
- **The supervisor's own death costs up to 0.5 s.** Move kept flowing for
  435 ms after the kill, bounded by the arbiter's `hold_timeout_sec`.
- **Sink death is uncovered.** Whether the GO2 stops when Move requests
  cease, and after how long, has not been measured on the robot. The fake
  GO2's 1 s Move timeout is not evidence about the robot.
- **Harness note: SIGKILL leaves stale discovery.** A SIGKILLed node's
  services stay in DDS discovery until its lease expires. A client can
  therefore see `change_state` as ready and send a request the restarted
  process never receives. The harness waits for an answered `get_state`
  first. Any restart tooling on the robot needs the same wait.

### Protective-latch regression (2026-10-18, simulation only)

`results/native_protective_latch_sim.json` records the regression campaign
against the localhost fake GO2. Physical motion was disabled; no lab host
or robot was accessed. Each case first verifies that healthy startup plus
streaming input sends no Move until explicit operator release.

The supervisor case first kills it while moving, then asserts a latched
critical fault before another SIGKILL during three supervisor restarts.
NOMINAL health and new epochs do not
release the arbiter latch. Exact old release messages, old challenges with
newer sequence numbers, and stale service authorizations are rejected.
Motion resumes only after a current operator authorization and new source
input. A supervisor lifecycle cleanup/configure/activate cycle also keeps
the latch and rejects its previous authorization. The detector case also rejects a release while unhealthy and rejects
that pre-health authorization after recovery. Core regressions additionally
cover heartbeat recovery without an intervening output tick, arbiter startup
and lifecycle reset, a new arbiter session, source-cache flushing, and
automatic supervisor clearance without operator authorization.

The release service now uses `helix_msgs/srv/ReleaseHold`. It binds the
operator action to the current supervisor epoch, supervisor health/state
revision and arbiter challenge. `ArbiterStatus.protective_latched` is the
authority for whether that release actually took effect. See
[`NATIVE_RUNTIME.md`](NATIVE_RUNTIME.md#arbiter-owned-protective-latch).

**Separate unresolved failure: sink SIGKILL.** The campaign confirms that
no process sends StopMove after the sink dies. Its result is explicitly
`UNRESOLVED_NO_STOP_PATH`, not a successful safety case. A surviving
watchdog/output path or a measured controller-side timeout is still needed.
The fake GO2's timeout proves nothing about the physical controller. The
supervisor-restart fix does not close this deployment blocker. The PR stays
draft; physical crash-recovery and motion validation remain unverified.

The original validation also showed the rate monitor exiting with SIGSEGV
during SIGINT cleanup. A follow-up reproduced it, identified a subscription
ownership cycle retaining DDS resources through RMW unload, and removed the
cycle while fixing timer cleanup. Repeated lifecycle/process and sanitizer
regressions pass; the new native campaign has no cleanup warnings. Original
results remain unchanged. The experimental independent Sport gate stops the
fake GO2 with command expiry disabled, but is not enabled in native launch.
See [runtime failure evidence and remaining limits](RUNTIME_FAILURES.md).

### Complete native Sport gate integration (simulation only)

[`native_sport_gate_integration_sim.json`](../results/native_sport_gate_integration_sim.json)
is a separate campaign with the real native rate monitor, anomaly detector, log
parser, supervisor, protective arbiter, sink and experimental gate, plus FakeGo2.
The fixed localhost launch uses private topics and test-owned communication
relays. FakeGo2 retains the last Move indefinitely, with expiry explicitly disabled.
All previous result files, including the original restart failure and uncovered
sink-crash result, remain unchanged.

The new profile routes final-gate feedback to the supervisor and requires fresh
controller replies and state independently of sink traffic. Its gate heartbeat
makes gate death visible to native supervision. Health recovery, new epochs and
repeated restarts preserve both latches. The test driver represents explicit
operator actions; it is not a production automatic recovery controller.

Gate death, final request-link loss and ineffective StopMove retain motion in the
simulation. The passing negative cases demonstrate a known failure boundary,
not a successful stop. A controller-side bounded motion lease or an independent
robot-side stop/inhibit path remains required. The GO2's actual controller timeout
is unknown. See [failure matrix, timings and reproduction](RUNTIME_FAILURES.md#complete-native-stack-integration).

## Missing experiments

Each of these is needed before the corresponding claim can move up a class:

1. **Stop from steady walking.** A trial in which the robot sustains a walk
   (several seconds of steady velocity, with `/lowstate` foot contacts
   showing steps) before the fault. Blocked on the gait investigation
   above. It may also need a speed limit above 0.25 m/s, which is an
   operator decision.
2. **No-HELIX counterfactual.** The same Move stream, for the same duration,
   with no hold, measuring travel. This is the denominator for any
   "HELIX prevented X cm of travel" statement. Not run.
3. **Repeat statistics.** At least 10 motion trials per condition. Current
   n is 2 passing motion trials.
4. **A real trigger.** A stop caused by a real degraded input (a covered or
   unplugged LiDAR, a killed process on the Orin) rather than an injected
   `FaultEvent`.
5. **Crash recovery on the robot.** The crash cases above ran in simulation
   only.
6. **An independent motion reference.** Every speed in this audit comes
   from the robot's own estimator. A second sensor (external tracking, a
   tape-measured mark, or a camera) is needed to confirm travel and stop
   distance.
7. **Final output authority loss.** Native deployment has no surviving
   StopMove path after sink death. The simulation gate covers upstream sink
   loss, but its own death or loss of its final request link retains motion.
   Whether the physical GO2 autonomously stops on command silence remains unknown.
8. **Soak and jitter.** The longest native run is 10 minutes.
