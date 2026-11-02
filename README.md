# HELIX

**Sensor-integrity monitoring and independent stop verification for the Unitree GO2, as one small observe-only ROS 2 node.**

[![CI](https://github.com/yusufdxb/helix/actions/workflows/ci.yml/badge.svg)](https://github.com/yusufdxb/helix/actions/workflows/ci.yml)
![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E)
![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C)
![License: MIT](https://img.shields.io/badge/license-MIT-green)

HELIX watches a GO2's sensor streams and answers two questions:

1. **Is a stream lying?** A stream that keeps its rate and stays in range can still be dead: a republished stale
   buffer, a frozen driver, a biased gyro. `ros2 topic hz`, `diagnostic_updater` frequency checks and range limits
   all pass on those. HELIX adds checks that look at content: frozen payloads, stalled stamps, and a cross-sensor
   residual between the body IMU and the LiDAR IMU.
2. **Did the robot actually stop?** When a StopMove (api 1003) is commanded, a checker that trusts the state
   estimator's velocity reports "stopped" even when the estimator is wrong. HELIX only says VERIFIED when at least
   two evidence groups agree, at least one of them a raw sensor (joint velocities or body gyro), and no group still
   shows motion. If the raw evidence is missing, it says UNVERIFIED instead of guessing.

HELIX never commands the robot. It has no publisher on any command topic. It reports; something else decides.

## Results (held-out, pre-registered)

The study was registered before any held-out data was scored
([preregistration](docs/preregistration/2026-11-02-helix-v2-content-faults-and-stop-verification.md)), the code was
frozen, and the held-out split was scored once. The full write-up, with every deviation, is in
[docs/results/2026-11-02-helix-v2.md](docs/results/2026-11-02-helix-v2.md).

| Claim | Outcome |
|---|---|
| **C1** HELIX detects content faults that rate and range baselines miss, at ≤ 2 false alarms per hour | **Killed by its false-alarm criterion.** Detection passed (+0.531 detection rate over the rate-or-range union on the registered fault pool, 95% CI [0.480, 0.579], positive in all 3 recordings), but HELIX raised 2 clean alarms in 0.449 h (4.5 /h). Both came from one real 49 ms publisher stall that was not labelled in advance. |
| **C2** Independent-evidence stop verification refuses stops the estimator fakes | **Supported.** Genuine stops VERIFIED 36/36. With the estimator velocity zeroed while the robot kept walking, HELIX refused 35/36; the estimator-only and ack-only checkers accepted 36/36. |

Selected detection numbers (episodes detected within 2 s, out of 40 per class, faults injected into `/utlidar/imu`):

| Fault | rate + range baselines | HELIX |
|---|---|---|
| dropout / rate collapse | 40, 40 | 40, 40 |
| frozen payload (stamp frozen / stamp live) | 0, 0 | **40, 40** |
| gyro z bias 0.2 rad/s | 4 | **40** |
| gyro z bias 0.1 rad/s | 4 | 4 |
| gyro noise σ 0.3 | 5 | **0** (worse: HELIX has no range check) |

A sham that misaligns the two IMUs by 20 s gives 0 detections in every class, so the residual depends on real
cross-sensor agreement.

The C++ node was also run against real bag playback (`eval/node_smoke.sh`): no alarms on a clean 79 s bag, UNVERIFIED
on a native stop bag that has no `/lowstate`, and frozen + stamp + rate raised on a constant-payload control.

## What is not shown

- **No external ground truth for stops.** No motion capture. Genuine stops assume the GO2 obeyed StopMove.
- **Synthetic faults.** All faults are injected at the message level into real recordings.
- **Short clean data.** 0.449 h cannot support any false-alarm rate below about 8 /h.
- **No recovery.** HELIX does not stop, restart or reconfigure anything, and no recovery is claimed. "VERIFIED" means
  the sensors agree that motion ended, not that HELIX caused it.
- **One robot.** Thresholds are calibrated on DEV recordings from a single GO2 and firmware.

## Layout

```
src/helix_monitor/   ROS-free core (checks, StopVerifier, config parser), the ROS 2 node, monitor_eval, gtests
src/helix_msgs/      FaultEvent, StopVerdict
eval/                bag extraction, calibration and scoring harness (Python), live-node smoke test
docs/                preregistration, results, GO2 field notes
hardware_eval_*/     raw GO2 recordings and earlier hardware evaluations
```

The same `helix_monitor_core` library runs in the node and in `monitor_eval`, the offline driver that the
evaluation harness feeds. Offline scores and live behaviour come from one implementation.

## Build

The core and `monitor_eval` need only a C++20 compiler and CMake. The ROS 2 node additionally needs the
`unitree_go` and `unitree_api` message packages from
[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2); without them only the core is built.

```bash
source /opt/ros/humble/setup.bash
source <unitree_ros2>/cyclonedds_ws/install/setup.bash   # or copy unitree_go, unitree_api into src/
colcon build --packages-up-to helix_monitor
colcon test --packages-select helix_monitor && colcon test-result --verbose
```

## Run

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch helix_monitor monitor.launch.py            # config:=/path/to/other.cfg to override
ros2 topic echo /helix/faults                          # helix_msgs/FaultEvent, raise and clear transitions
ros2 topic echo /helix/stop_verdict                    # helix_msgs/StopVerdict, one per StopMove burst
```

Subscribed: `/utlidar/imu`, `/sportmodestate`, `/lowstate`, `/api/sport/request` (read only, to see StopMove).

**Use Cyclone DDS.** The GO2 publishes `SportModeState` samples of 516 bytes while the `unitree_go` type serializes
to 236. Cyclone accepts them; Fast DDS (the Humble default) drops them silently, so the monitor would see no
`/sportmodestate` at all. The node warns at startup on any other RMW, and again if a monitored stream is silent for 5 s.

The deployed checks and thresholds are plain text in
[src/helix_monitor/config/go2.cfg](src/helix_monitor/config/go2.cfg), generated by the calibration step of the harness.

## Reproduce the evaluation

```bash
export HELIX_NPZ=/path/to/npz        # see eval/DATA.md for the recordings and their sha256
python3 eval/helix_eval.py score --split heldout
eval/node_smoke.sh <clean_bag> <native_stop_bag>   # live node, localhost-only DDS, private domain
```

The native-stop bags are tracked in `hardware_eval_20261017/`; the other recordings are listed with hashes in
[eval/DATA.md](eval/DATA.md). Rescoring with the committed code reproduces `eval/out/results_heldout.json`.

## History

HELIX v1 was a larger fault-detection and self-healing stack (anomaly scoring, diagnosis, an LLM fault explainer, a recovery
node, a dashboard). v2 removed it: the parts that held up were rebuilt as this monitor, and recovery was dropped because it
could not be verified. The v1 code is at tag `v0.3.0-wip-self-healing` and commit `86e4e60`.

## Author

**Yusuf Guenena**
M.S. Robotics Engineering, Wayne State University
[linkedin.com/in/yusuf-guenena](https://linkedin.com/in/yusuf-guenena) &middot; [github.com/yusufdxb](https://github.com/yusufdxb)

## License

MIT. See [LICENSE](LICENSE).
