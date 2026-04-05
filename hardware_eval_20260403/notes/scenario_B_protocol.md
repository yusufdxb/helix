# Scenario B: Node Lifecycle Observation

## Protocol
**Objective**: Document the observability of GO2 node lifecycle events from external machines.

**Safety classification**: OBSERVATION ONLY -- no nodes were killed, restarted, or modified.

### Steps Executed
1. Ran `ros2 node list` from PC (default DDS config) -- returned EMPTY
2. Ran `ros2 node list` from Jetson (default DDS config) -- returned EMPTY
3. Ran `ros2 node list` from Jetson with rmw_cyclonedds_cpp -- FAILED (CycloneDDS not
   installed on Jetson)

### Results
- **Node discovery is NOT available from external machines.** Neither the PC nor the Jetson
  can enumerate GO2 nodes via `ros2 node list`.
- **Topic discovery DOES work.** Both machines can see all GO2 topics and their types.
- This asymmetry suggests the GO2 uses a DDS configuration that exposes topic endpoints but
  restricts node-level participant discovery (possibly via DDS domain isolation or a custom
  DDS XML config on the robot).

### Nodes That Theoretically Could Be Observed
Based on the topic namespace structure, the GO2 likely runs these node categories:
- **utlidar/** -- LiDAR processing (cloud, odom, SLAM, height map, voxel map)
- **uslam/** -- SLAM frontend and localization
- **lio_sam_ros2/** -- LIO-SAM mapping
- **api/** -- Service API endpoints (sport, motion_switcher, obstacles_avoid, etc.)
- **lf/** -- Low-frequency state monitors (battery alarm, lowstate, sportmodestate)

### Implications for HELIX
- **HELIX cannot use ros2 node list to detect node death from external machines.**
- Instead, HELIX must infer node health from topic liveness:
  - If /utlidar/imu stops publishing (currently 249 Hz), the LiDAR node has likely died
  - If /utlidar/robot_odom drops (currently 150 Hz), the odometry pipeline has failed
  - Rate drop thresholds should be calibrated from the baseline bags
- Alternative: SSH into the Jetson or robot to run `ros2 node list` locally, but this
  adds latency and a dependency on SSH connectivity

### Evidence
- No bag recorded (observation-only scenario)
- Topic list captured in baseline_001 bag metadata
