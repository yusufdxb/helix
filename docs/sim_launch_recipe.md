# Sim launch recipe: green physical-closure gate from a cold machine

`tests/sim_integration/test_lidar_occlusion_recovery.py` is the only HELIX test
that claims *physical* closure: the robot was moving, the loop stopped it, and
odometry proves it. It skips unless a simulator is publishing `/utlidar/cloud`
and `/utlidar/robot_odom`. This file is the exact sequence that makes it run.

Nothing here weakens the gate. The skip guard and every assertion in
`scripts/validate_closed_loop_bag.py` are untouched.

## Status as of 2026-08-07: still skipping, one blocker left

Verified working on mewtwo (Isaac Sim 6.0, IsaacLab 4.5.22, RTX 5070):

- Isaac Sim boots headless with the GO2 scene and reaches its main loop.
- Cross-runtime DDS works: Isaac's bundled Humble and system Humble discover
  each other on `ROS_DOMAIN_ID=41` over `rmw_cyclonedds_cpp`.
- `robot0/odom` publishes at 108 to 133 Hz, and the bridge republishes it as
  `/utlidar/robot_odom` with a reconstructed twist.
- The command path drives the body: an 0.20 m/s command on `/cmd_vel` produced
  a measured 0.355 to 0.575 m/s, and an explicit zero decayed it to 0.004 m/s.
  Both odometry halves of the gate (>= 0.08 moving, <= 0.03 stopped) are
  physically achievable, with margin.

Blocker: **the RTX LiDAR attaches but its scan buffer stays empty**, so
`robot0/point_cloud2` never publishes and the gate skips on `/utlidar/cloud`.
`add_rtx_lidar` reports `rtx lidar attached (config=Hesai_XT32_SD10)` and the
annotator resolves, but `IsaacCreateRTXLidarScanBuffer` returns shape `(0,)`
every frame. Leading hypothesis, not yet confirmed: the manually created
replicator render product is never ticked, because IsaacLab's `env.step()`
drives only the sensors IsaacLab itself manages. The next thing to try is
forcing a render each step, or registering the LiDAR as an IsaacLab sensor so
its render product is on IsaacLab's update path.

Do not work around this by synthesizing a point cloud in the bridge. The gate
would go green on a sensor that was never alive, which is worse than the skip.

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

The repo's own `Isaac_sim/Unitree/Unitree_L1.json` is still written against the
pre-5.0 profile schema (`profile.emitters` dict; Isaac 5/6 want
`profile.emitterStates` plus `emitterStateCount`) and is **not** usable. Until
someone migrates it, pass a config that ships with Isaac. `Hesai_XT32_SD10` is
what this recipe uses: a local single-prim JSON config, no remote-USD variant
lookup. It is a stand-in for the GO2's real Unitree L1. The gate is a
*rate*-based staleness test, so the sensor's beam pattern does not affect the
result, but do not read the point geometry as GO2-accurate.

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
