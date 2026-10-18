# Supervised walking-stop experiment

Status: prepared, not executed or authorized for motion. The installed firmware,
controller silence behavior, separate emergency path and current ARM deployment
remain unverified. [Controller inspection](CONTROLLER_INSPECTION.md) records that
boundary. No software watchdog layer is added by this protocol or its analyzer.

## Claim and prerequisites

The intended claim is narrow: an injected HELIX fault, during independently
observed sustained straight walking, produces a latched hold, no subsequent Move
commands and an externally measured stop. It is a commanded-stop experiment with
a synthetic trigger, not proof of real sensor-fault detection or safety certification.

Before a physical trial, all of the following must have evidence for the same
candidate, robot configuration and session:

1. Identify the installed robot firmware/motion build and current mode through a
   vendor-supported read-only report. Separate these from payload OS and service
   API versions. Establish the applicable command-silence contract with the vendor,
   or explicitly retain it as unknown and provide validated containment for it.
2. Build and test the native candidate on the payload, including every consumer of
   the updated message/service ABI. Complete a ten-minute non-actuating capture
   with real sensor traffic, no Move/StopMove sent by HELIX, a retained protective
   latch, clean teardown, CPU/RSS and thermal measurements under the intended load.
   Agree resource and timing budgets before the motion campaign; historical values
   are reference measurements, not acceptance of the new candidate.
3. Establish a vendor-supported separate operator stop on this robot while it is
   physically supported. In a separately authorized test, verify it while the
   payload/Sport sender is unavailable and record the observed result and response
   time. The handheld's documented damping action changes mode and can cause a
   fall. It has not been exercised in this follow-up. Do not infer operation from
   its button mapping or silence on `/wirelesscontroller`.
4. Have a qualified lab operator approve a rated restraint/fall-arrest arrangement
   and exclusion area that contain motion even if neither command path stops it.
   The arrangement must not prevent ordinary foot contact or create a tether-induced
   false stop. Retain its setup and limits in session evidence. No person catches
   or reaches under a moving robot. No improvised battery removal contingency.
5. Provide a genuinely independent tracking source, calibrated and synchronized as
   below, with an independent view of alternating foot contacts. `/sportmodestate`
   and `/utlidar/robot_odom` are one onboard estimate and do not qualify.
6. Demonstrate sustained walking within the existing 0.25 m/s sink limit, unchanged
   controller mode and travel guard. The earlier low-speed trials did not establish
   this. If the current gait cannot walk within these bounds, stop the campaign.
   A higher speed limit, gait or controller-mode change needs a separate decision
   and validation, not an automatic adjustment during a stop trial.

## Independent measurement

Prefer an external calibrated motion-capture system tracking body position and yaw,
plus fixed video showing feet. A fixed camera with a body marker is acceptable only
if its pose calibration, distortion, marker height/parallax and occlusion uncertainty
meet the same bounds. A floor tape alone does not establish velocity or stop timing.

Record raw tracking/video, calibration, marker placement, sample rate, dropped frames
and the conversion to SI units. Use at least 120 Hz for camera tracking. Bound position
error to 2 mm and yaw error to 0.004 rad. These are required measurement budgets,
not claims about equipment currently available in the lab.

Map the injected fault's host-monotonic timestamp into the external capture clock
using a shared hardware timestamp or a calibrated visible synchronization signal.
Record at least three synchronization events spanning the trial, the mapping and
residuals. Include frame quantization and the measured signal/transport delay in a
total uncertainty bound of at most 20 ms. Do not align clocks by subtracting unrelated
wall-clock dates or assume a USB/display event has zero delay. If the bound cannot
be established, timing is unverified. Preserve the raw events for reanalysis.

Export tracking CSV with columns `t_s,x_m,y_m,yaw_rad,valid`; `valid` is 1 or 0 and
time is strictly increasing in the external clock. The paired metadata JSON contains:

| Field | Meaning |
|---|---|
| `tracking_source` | `external_camera` or `external_mocap`; GO2 estimates are rejected |
| `fault_t_s` | injected fault mapped into the external clock |
| `sync_uncertainty_s` | total established synchronization bound, at most 0.020 s |
| `position_uncertainty_m` | established position bound, at most 0.002 m |
| `yaw_uncertainty_rad` | established yaw bound, at most 0.004 rad |
| `walking_cycles_before_fault` | complete alternating gait cycles annotated from independent video |
| `raw_tracking_sha256`, `calibration_sha256`, `gait_video_sha256` | hashes of retained source artifacts |

The offline analyzer requires two seconds of uninterrupted pre-fault body velocity
bounded between 0.10 and 0.25 m/s, plus at least two annotated gait cycles. Body sway
or a short velocity spike cannot qualify. It uses a 200 ms trailing displacement
window, adding `2 * position_error / interval` to the speed bound and the analogous
yaw bound. For stillness it checks every sampled secant ending at each frame with
a lag of 100-200 ms, so a return to the same position every 200 ms cannot hide
oscillation. A still interval requires every resulting upper secant-speed bound
at most 0.05 m/s and yaw bound at most 0.10 rad/s for 0.5 seconds, followed by stillness through at
least three seconds after the fault. Invalid frames and gaps above 50 ms break the
measurement. The trailing window delays the criterion; report that delay and clock
uncertainty rather than presenting its timestamp as exact mechanical cessation.
Sampled secants cannot bound motion hidden between frames; review raw video and
tracking artifacts as well as the numerical criterion.

```bash
python3 scripts/hw/external_stop_analysis.py --track track.csv \
  --metadata measurement.json --out external_stop.json
```

The analyzer has no robot I/O and is not an online protective controller. It cannot
authenticate supplied tracking or gait annotations. Its output deliberately leaves
the overall hardware-trial verdict unevaluated and never authorizes motion.
`tracking_source=synthetic` is reserved for software tests and labels their output
Synthetic. No synthetic file may satisfy the hardware prerequisites.

## One trial, two operators, no automatic release

The motion operator handles the native stack and explicit current arbiter release.
The safety operator holds the validated independent stop control, watches the robot
directly and can terminate the trial without waiting for the command computer.
Both confirm the corridor, containment, sensor visibility and contingencies before
each trial. Keep the onboard state estimates as diagnostics, not the primary motion
reference. Do not change controller mode, acquire a Sport lease or run vendor motion
examples as part of trial setup.

Start the exact clean native candidate with its documented armed hardware profile
only after the physical prerequisites and separate motion authorization. Record the
full bag before arming: Sport requests/responses, hold, supervisor, arbiter, sink,
source/output velocity, faults, `/lowstate`, both onboard motion estimates and
`/wirelesscontroller`. Preserve the external recording separately. Capture the live
graph and platform publisher baseline; stock Sport publishers are not automatically
foreign commands. Investigate actual request identities and activity.

Authorize startup through `ReleaseHold` using fresh supervisor epoch/revision and
the current arbiter challenge. Confirm the latch actually cleared. Never replay or
refresh a rejected request automatically. No source may stream before the operator
chooses to begin the bounded trial.

The existing runner can perform one injected-fault trial while leaving the hold
active. The future motion invocation is:

```bash
python3 scripts/hw/native_stop_test.py --stage motion --out trial \
  --vx 0.20 --settle-s 2.1 --hold-stream-s 3.0 --stop-deadline-s 1.5 --keep-held
```

This is a future hardware command, not part of inspection or validation performed
here. It neither changes gait nor raises limits. It starts only with the arbiter
already explicitly released. The runner's onboard-estimator moving condition is
only an execution condition: the independent analysis decides whether the trial
actually contained sustained walking. If the existing 0.6 m travel guard or any
other abort trips, record FAIL and retain the trace; do not relax it to obtain PASS.
Confirm that the planned walking interval and stopping margin fit the approved
travel budget before beginning.

At the injected fault, retain upstream streaming long enough to test the hold's
authority, unless an abort/contingency requires termination. The runner sends zero
at the end and `--keep-held` leaves the injected fault and protective latch active.
No clear or release occurs in that mode. A fresh human operator decision is required
to clear/release after the robot is independently still and the source is silent.
Do not use the runner's legacy automatic finish-release mode for this experiment.

## Final-command-path contingency

Predeclare a maximum 1.5-second external stop deadline and a shorter emergency
decision time consistent with the validated separate stop response and remaining
contained travel. Select that decision time from measured emergency-path latency
and the approved speed/space budget before a trial; it is not measured here.

On missed stop publication/acknowledgment, continued external motion, loss of tracking,
unexpected command authority, overspeed or approach to the containment boundary:

1. The safety operator invokes the separately validated handheld emergency action
   immediately according to its agreed trigger. Treat any intervention as an
   aborted/confounded trial, not a HELIX stop success.
2. If that action does not halt the robot, the independently approved physical
   containment must prevent escape/fall without a person approaching it. Isolate
   the area and follow the vendor/lab recovery procedure. A failed onboard controller
   may also defeat the handheld; the protocol does not claim otherwise.
3. Preserve command, external motion and intervention records. No restart or
   automatic rearm is allowed. Investigate before another trial.

Stopping the source, Ctrl-C, a shutdown StopMove burst and another Sport publisher
all depend on the final command path. They are normal cleanup, not this contingency.
If the separate stop and physical containment cannot be validated, physical motion
is NO-GO. Do not kill the sink, final gate or controller link while walking to
investigate timeout before these provisions are established. Any future silence
characterization is a separate supervised test, not hidden inside the nominal trial.

## Separate future controller-silence characterization

This is not executed or authorized here. Complete the supported emergency-path
test, containment approval, external calibration and nominal walking-stop pilot
first. Obtain separate approval for intentionally losing the final sender while
walking. Keep the experimental software gate simulation-only. Identify the actual
firmware/mode, every Sport publisher and any automatic restart before the trial.
The trial is invalid if another publisher refreshes Move or a surviving process
sends StopMove before the measured stop.

Use the same bounded speed, travel guard, independent recording and two operators.
Once external tracking establishes sustained walking, terminate only the test's
identified final sink process without running its graceful StopMove cleanup.
Record the last observed Move, process exit and all subsequent Sport requests,
responses and external motion. Retain the native protective latch; neither a
restarted sink nor recovered health may release it. Disable automatic process
respawn for this separately supervised test. Do not restart the sink until the
source is silent, the robot is independently still and the latch is confirmed.
The independently validated operator stop and physical containment must cover the
interval when no final sender exists. Intervention is a censored timeout result,
never evidence that the controller stopped by itself.

Report whether motion ceased before intervention, the externally observed
last-Move-to-still interval, uncertainty, distance and repeat variability. A host
publication timestamp is not a controller receipt timestamp: without a bounded
transport delay or controller instrumentation, this is observed behavior after
publisher loss, not an exact firmware timer value. A matching response does not
prove when the velocity target was applied. Record posture/rearm behavior without
sending a delayed Move to probe it during the same trial.

One sender-death case does not establish behavior under payload power loss,
communication loss, queued commands or onboard-controller failure. Those require
separate approved conditions and the same contingency. Do not unplug an unknown
link that may also remove the fallback. A vendor-specific timeout contract still
needs to explain which failures it covers; finite successful trials cannot supply
an unconditional bound. Preserve every failed, aborted and negative result.

## Trial verdict and demonstration readiness

A walking-stop trial requires all of: verified external walking, complete fault to
hold/arbiter/StopMove trace, a matching controller reply, externally bounded stop
within 1.5 seconds including synchronization uncertainty, continued stillness, no
Move while held, no unexplained competing command, no intervention and retained
protection. Acknowledgment alone and the runner's internal PASS are insufficient.
An unexpected real fault, tracking gap, missing record or fallback intervention
makes the trial failed or inconclusive, with all evidence retained.

First complete one supervised pilot. A repeatable demonstration campaign then needs
at least ten valid trials at the same configuration, publishing every attempt and
latency/distance distributions. These repetitions do not establish a safety-rated
stop guarantee. Firmware identity, ARM validation, independent calibration, separate
emergency-path validation, containment, sustained walking at the existing limit and
explicit physical authorization are all still required before the first pilot.
