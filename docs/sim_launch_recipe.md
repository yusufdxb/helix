# Sim launch recipe: green physical-closure gate from a cold machine

`tests/sim_integration/test_lidar_occlusion_recovery.py` is the only HELIX test
that claims *physical* closure: the robot was moving, the loop stopped it, and
odometry proves it. It skips unless a simulator is publishing `/utlidar/cloud`
and `/utlidar/robot_odom`. This file is the exact sequence that makes it run.

Nothing here weakens the gate. The skip guard and every assertion in
`scripts/validate_closed_loop_bag.py` are untouched.

## Status as of 2026-08-08: the gate PASSES

Physical closure is proven in simulation. The robot is driven, its LiDAR is
dropped, HELIX detects it, decides, and the arbitrated zero command stops the
body, with odometry proving both the motion and the stop.

```
2 passed in 86.27s

PASS  zero-rate injection marker recorded
PASS  mux commanded nonzero motion before injection      nonzero_samples 49
PASS  stale LiDAR fault detected within bound            7.626 s (bound 10.0)
PASS  STOP_AND_HOLD decision follows the same fault      0.00029 s (bound 0.25)
PASS  recovery envelope accepts the same stop decision   0.00016 s (bound 0.25)
PASS  recovery publishes a zero command                  0.031 s (bound 0.25)
PASS  mux output carries the zero command                0.031 s (bound 0.50)
PASS  odometry proves physical motion before injection   0.223 m/s (needs >= 0.08)
PASS  odometry proves the robot stopped                  0.0104 m/s (needs <= 0.03)
```

Topic rates measured on this run: `/utlidar/cloud` 14.4 Hz, `/utlidar/robot_odom`
63.6 Hz, `/utlidar/imu` 71.3 Hz, `/utlidar/robot_pose` 68.2 Hz.

### What closed it

Two changes, neither of which relaxes anything the gate asks for.

1. The bridge now republishes `robot0/imu` as `/utlidar/imu`, and the pose half
   of `robot0/odom` as `/utlidar/robot_pose`. Both come from real simulator
   data. Without them SENSE saw two permanently stale topics, put HELIX into
   STOP_AND_HOLD before the scenario began, and the gate could never observe
   the robot moving.
2. `config/helix_adapter_params_sim.yaml` declares only the topics the
   simulator actually publishes. `gnss` and `multiplestate` have no simulator
   source at all, so the sim profile does not watch them. They remain in the
   hardware profile. This is a configuration difference, not a suppressed
   fault: adapter coverage is exactly what its YAML lists, and declaring a
   topic nothing publishes does not make HELIX more thorough, it makes it
   permanently wrong. `sim_mode:=true` selects the profile.

### One number worth watching

`injection_to_fault` is 7.626 s against a 10 s bound. That is the time for a
completely silent LiDAR to be called stale, and it is the least comfortable
margin in the table. See `analysis/README.md`: detection of sustained
degradation, as opposed to outright topic death, is where the shipped detector
config is weakest.

### An earlier failure worth remembering

Before the fix, `hint_to_accept` measured 4.497 s against a 0.25 s bound. That
was not an envelope defect and not a simulator timing artifact. Gaps between
accepted actions in that bag were 4.999, 5.005, 4.998, 5.001 and 5.005 s: the
per-fault cooldown. A hint arriving mid-cooldown waits for the window to
expire, so that bound only means anything for the first stop of an episode. On
this clean run the same measurement is 0.16 ms.

## What the bridge exists for

go2_omniverse does not speak GO2 topic names, and it does not fill in
everything the gate reads:

| What HELIX expects | What go2_omniverse does | Gap |
| --- | --- | --- |
| `/utlidar/cloud` | publishes `robot0/point_cloud2` | name only |
| `/utlidar/robot_odom` | publishes `robot0/odom` | name, **and `twist` is never populated** |
| subscribes muxed `/cmd_vel` | subscribes `robot0/cmd_vel` | name only |

The odometry row is why `scripts/sim_bridge/go2_sim_bridge.py` is a node and
not a set of launch remappings. `RobotBaseNode.publish_odom` in
go2_omniverse's `ros2.py` writes only the pose half of `nav_msgs/Odometry`
(`grep -n twist ros2.py` returns nothing), while the validator reads
`twist.twist.linear` to decide whether the robot was moving before the fault
and stopped after the recovery. Relayed verbatim that field is identically
zero, so both odometry checks would fail no matter how correctly the robot
moves. The bridge reconstructs twist by differentiating the simulator's own
ground-truth pose over a 0.20 s window. That is the same quantity a real
odometry node reports; it supplies a field the simulator omits rather than
relaxing anything the gate asks for.

## Prerequisites, one time

go2_omniverse needs three fixes before its LiDAR produces anything on
Isaac Sim 6.0 / IsaacLab 4.5.22. They are applied in
`~/workspace/go2_omniverse` (branch `feat/jazzy-isaacsim5-experimental`):

1. `omniverse_sim.py` hardcoded `annotator_lst = []`, so `add_rtx_lidar` was
   never called and `robot0/point_cloud2` could never publish. It is now opt-in
   behind `--lidar_config`.
2. `add_rtx_lidar` built on `isaacsim.sensors.rtx.LidarRtx`, which derives from
   `isaacsim.core.prims`, whose `prim.py` calls
   `SimulationManager._get_backend_utils()`. IsaacLab 4.5.22 swaps
   `SimulationManager` for its `PhysxManager`, which does not implement that
   method, so the class cannot construct. It now uses the low-level
   `IsaacSensorCreateRtxLidar` command plus `rep.create.render_product`.
3. Isaac Sim 6.0 renamed the scan-buffer annotator from
   `RtxSensorCpuIsaacCreateRTXLidarScanBuffer` to
   `IsaacCreateRTXLidarScanBuffer`, and `update_meshes_for_cloud2` still called
   `.cpu().numpy()` on buffers that the IsaacLab 4.5 port had already converted
   to numpy.
4. The annotator must be fetched as
   `IsaacExtractRTXSensorPointCloudNoAccumulator`, not
   `IsaacCreateRTXLidarScanBuffer`. See "per-frame output" below.
5. Clouds are decimated to at most 8192 points before publishing. See
   "message size" below.

### Config name casing: use `Hesai_XT32_SD10`, not `HESAI_XT32_SD10`

This is the single easiest thing to get wrong, and getting it wrong produces a
sensor that looks attached and is not the one you asked for.

There are two different resolution paths inside `IsaacSensorCreateRtxLidar`,
and the name selects between them:

- `Hesai_XT32_SD10` matches the **local JSON profile** shipped at
  `exts/isaacsim.sensors.rtx/data/lidar_configs/HESAI/Hesai_XT32_SD10.json`.
  No network. This is the one to use.
- `HESAI_XT32_SD10` matches a `SUPPORTED_LIDAR_CONFIGS` entry, which is a
  **remote USD** fetched from `get_assets_root_path()`. On this machine that
  root resolves to `.../Assets/Isaac/4.5/...`, which returns **HTTP 404**, and
  Kit logs `No OmniLidar prim found in referenced asset`. Every entry in
  `SUPPORTED_LIDAR_CONFIGS` is remote, and none of them resolve here.

`do()` is `_add_reference() or _call_replicator_api() or _create_camera_prim()`
and the last of those always returns a prim, so a wrong name never surfaces as
an exception. It silently yields Replicator's default rotary profile instead.
Both wrong names produced an identical ~204k-point generic cloud during this
work, which is exactly what a healthy sensor looks like from the outside.
Expect the warning `Config 'Hesai_XT32_SD10' not found` on the correct path:
that is `_add_reference` declining before the local-JSON path takes over, and
it is benign. The error you must not see is `No OmniLidar prim found`.

The repo's own `Isaac_sim/Unitree/Unitree_L1.json` is still written against the
pre-5.0 profile schema (`profile.emitters` dict; Isaac 5/6 want
`profile.emitterStates` plus `emitterStateCount`) and is **not** usable. The
HESAI is a stand-in for the GO2's real Unitree L1. The gate is a *rate*-based
staleness test, so the sensor's beam pattern does not affect the result, but do
not read the point geometry as GO2-accurate.

### Per-frame output, and why `initialize()` does not do it

`annotator.initialize(enablePerFrameOutput=True)` on
`IsaacCreateRTXLidarScanBuffer` is silently ignored. In Isaac 6.0's
`isaacsim.sensors.rtx` extension, `_register_nodes` registers that name with no
`init_params` at all, and registers a *second* annotator,
`IsaacExtractRTXSensorPointCloudNoAccumulator`, over the same OGN node type
with `init_params={"enablePerFrameOutput": True}` baked in. Fetch that second
name. Fetching the first and calling `initialize` measured 195k to 204k points
per `get_data()` either way, which is full-revolution accumulation.

### Message size: the cap is not cosmetic

Even on the per-frame annotator, one `get_data()` returns 195k to 204k points,
because a rendered frame spans ~0.3 s of wall clock here and the RTX sensor
traces continuously across it. That is ~2.4 MB per `PointCloud2`, and at that
size the messages never arrive: `ros2 topic hz /robot0/point_cloud2` received
**zero messages in 20 s** while the publisher was running, and the publish cost
dragged `/robot0/odom` from 108-133 Hz down to 1.8 Hz. Accumulation-sized
clouds do not merely waste bandwidth, they stall the sim's main loop.

`ros2.py` therefore strides the returns down to at most 8192 points (~98 KB).
This decimates real returns, it does not synthesize them: every published point
is a ray the sensor actually traced that frame, and if the sensor stops there is
nothing to stride over and nothing is published, so the staleness gate still
sees a genuinely dead topic.

A related pre-existing bug is fixed alongside it: `pub_robo_data_ros2` took
`start_time` by value and rebound it locally, so the caller kept passing its
original value and the `1/20` cadence check was true on every physics step.
The cadence now lives in module state.

## The sequence

Everything runs on `ROS_DOMAIN_ID=41` over `rmw_cyclonedds_cpp`. Both ends must
match on both, or the topics simply never meet.

### Terminal 1: Isaac Sim + GO2 scene

```bash
cd ~/workspace/go2_omniverse
export ROS_DOMAIN_ID=41
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
./run_sim_humble.sh --headless --enable_cameras --lidar_config Hesai_XT32_SD10
```

Measured latencies from the run of 2026-08-08, for reference when comparing a
later run (all five are recorded even when the gate fails):

| Latency | Measured | Bound |
| --- | --- | --- |
| `injection_to_fault` | 7.626 s | 10.0 s |
| `fault_to_hint` | 0.00029 s | 0.25 s |
| `hint_to_accept` | 0.00016 s | 0.25 s |
| `accept_to_helix_zero` | 0.031 s | 0.25 s |
| `accept_to_mux_zero` | 0.031 s | 0.50 s |

Every one is inside its bound. The one to watch is `injection_to_fault` at
7.626 s against 10 s: calling a fully silent topic stale is the slowest link in
the chain, and it is the same weakness `analysis/README.md` measures from the
other direction.

Notes on the flags and env:

- `run_sim_humble.sh` already sets `OMNI_KIT_ACCEPT_EULA=YES`, activates
  `~/Sim/isaac-sim-venv`, and points `PYTHONPATH` / `LD_LIBRARY_PATH` at Isaac's
  bundled Humble. Do **not** source `/opt/ros/humble` in this terminal: mixing an
  external rclpy with the bundled one causes typesupport double-loads.
- The launcher defaults `RMW_IMPLEMENTATION` to fastrtps but honors a pre-set
  value, so exporting cyclonedds before calling it is enough. No edit needed.
  Isaac's bundled Humble ships `librmw_cyclonedds_cpp.so`, so this works.
- `--enable_cameras` is required: the RTX LiDAR is a render-product sensor and
  produces nothing headless without it.
- Wait for `[go2_omniverse] rtx lidar attached` and `entering main loop`.
  If you see `rtx lidar FAILED`, the LiDAR is dead and any resulting green gate
  would be an artifact, not a result.

### Terminal 2: the topic-name bridge

```bash
cd ~/workspace/helix
export ROS_DOMAIN_ID=41
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/sim_bridge/go2_sim_bridge.py
```

It logs a relay count and the current reconstructed speed every 5 s. Non-zero
and rising `cloud=` / `odom=` counters mean the sim side is healthy.

### Terminal 3: verify before trusting anything

```bash
export ROS_DOMAIN_ID=41
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
ros2 topic hz /utlidar/cloud
ros2 topic hz /utlidar/robot_odom
```

Both must report a steady non-zero rate. `/utlidar/cloud` at 0 Hz means the
LiDAR never attached, and the gate would then "detect" a stale topic that was
never alive.

### Terminal 3: run the gate

```bash
cd ~/workspace/helix
export ROS_DOMAIN_ID=41
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest tests/sim_integration/test_lidar_occlusion_recovery.py -v
```

The test runs `scripts/sim_faults/run_closed_loop_scenario.py` itself, which
brings up `helix_closedloop.launch.py` (with `sim_mode:=true`,
`enable_twist_mux:=true`, recovery auto-activated), records an MCAP bag, drives
`/nav/cmd_vel` at 0.20 m/s, and injects the LiDAR rate drop.

To run the scenario by hand and inspect the bag:

```bash
python3 scripts/sim_faults/run_closed_loop_scenario.py \
    --duration 75 --schedule 20:10,20:0,30:10 --nav-speed 0.20 \
    --artifact-dir results/sim_run_manual
python3 scripts/validate_closed_loop_bag.py results/sim_run_manual/bag
```

## Shutdown

Kill the processes you started, by PID. Do not use broad patterns like
`pkill -f helix`: other work commonly runs against this tree.

## Signal path

```
Isaac Sim (go2_omniverse, bundled Humble)
  robot0/point_cloud2 ─┐
  robot0/odom ─────────┤
                       ▼
             go2_sim_bridge.py  (renames; reconstructs odom twist)
                       │
   /utlidar/cloud ─────┴──► inject_lidar_rate_drop.py ──► /utlidar/cloud_throttled
   /utlidar/robot_odom ────► HELIX adapter + closure validator
                                        │
   /nav/cmd_vel ──► twist_mux ──► /cmd_vel ──► go2_sim_bridge.py ──► robot0/cmd_vel
                        ▲
   /helix/cmd_vel ──────┘  (recovery STOP_AND_HOLD, priority 100)
```

Routing twist_mux's muxed `/cmd_vel` (not `/helix/cmd_vel` directly) into the
simulator is what makes the "robot stopped" half of the gate meaningful: the
zero that reaches the body is the arbitrated one.
