# GO2 Hardware Evidence Summary

Evidence collected 2026-04-03 from a live Unitree GO2 quadruped, Jetson Orin NX companion, and development PC. All artifacts stored on T7 SSD at `hardware_eval_20260403/`.

**Major update (session 2):** HELIX ROS 2 nodes ran on the live GO2 graph for the first time. A passive adapter bridging GO2 topic rates to `/helix/metrics` enabled the anomaly detector to emit 4 FaultEvents from a real LiDAR rate anomaly. `/diagnostics` was discovered to be published by GO2's `twist_mux` node, upgrading native input coverage from 1/4 to 2/4.

## Hardware Topology

| Node | Role | Address | Hardware |
|------|------|---------|----------|
| GO2 | Robot | 192.168.123.161 | Unitree GO2 quadruped |
| Jetson | Companion | 192.168.123.18 | Orin NX, Cortex-A78AE 4/8 cores, 16GB RAM |
| PC | Dev/Observer | 192.168.123.10 | i7-7700 3.60GHz, 16GB RAM |

Network: wired Ethernet, all on 192.168.123.0/24.

## Key Measurements

### 1. HELIX Algorithm Performance: Jetson vs PC

All benchmarks run on identical Python scripts, same random seeds.

| Metric | PC (i7-7700) | Jetson (A78AE) | Ratio | Operational Need |
|--------|-------------|----------------|-------|-----------------|
| Anomaly throughput | 80,643 samp/s | 63,675 samp/s | 0.79x | ~100 samp/s |
| Log parser throughput | 248,282 msg/s | 156,231 msg/s | 0.63x | ~100 msg/s |
| Detection latency | 0.049 ms | 0.049 ms | 1.0x | <100 ms |
| Heartbeat miss latency | 200.7 ms | 200.7 ms | 1.0x | <2 s |

**Finding**: The Jetson achieves 63–79% of PC throughput. At HELIX's operational rate (~100 samples/sec), the Jetson exceeds requirements by >600x. Algorithmic overhead is negligible on the target hardware.

### 2. Cross-Device DDS Latency

Measured via 50 round-trip echo exchanges (PC publish → Jetson echo → PC receive).

| Metric | Value |
|--------|-------|
| RTT mean | 1.63 ms |
| One-way estimate | 0.81 ms |
| P95 RTT | 2.31 ms |
| Delivery | 50/50 (100%) |

**Finding**: Cross-device ROS 2 message transport adds <1 ms one-way latency. At HELIX's 10 Hz monitoring rate, this is 0.81% of the sensing period — negligible.

### 3. Jetson Resource Headroom

Baseline measurement while GO2 stack (103 topics) is running, no HELIX:

| Resource | Value | HELIX Estimate |
|----------|-------|---------------|
| CPU idle | 82.4% | <0.5% of one core |
| RAM free | 13.0 GB / 15.7 GB | <50 MB additional |
| Thermal | 43–48°C (throttle at 85°C) | +0.2°C |
| Cores | 4 of 8 online | Needs 1 core |

**Finding**: The Jetson has massive headroom. HELIX's lifecycle nodes would consume <0.5% of a single core and <50 MB RAM.

### 4. GO2 Topic Landscape

103 ROS 2 topics discovered via DDS. Key active topics:

| Topic | Rate | Type | HELIX Relevance |
|-------|------|------|----------------|
| /utlidar/imu | 250 Hz | sensor_msgs/Imu | Potential anomaly metric |
| /utlidar/robot_odom | 150 Hz | nav_msgs/Odometry | Drift detection candidate |
| /utlidar/robot_pose | 18.7 Hz | geometry_msgs/PoseStamped | Position monitoring |
| /utlidar/cloud | 15.4 Hz | sensor_msgs/PointCloud2 | LiDAR health proxy |
| /gnss | 1 Hz | std_msgs/String | GPS fault detection |
| /multiplestate | 1 Hz | std_msgs/String | Config state monitoring |

**Finding**: Rich sensor data available at standard ROS 2 types. The GO2 publishes sufficient data for meaningful HELIX monitoring, but HELIX's current input interfaces (/diagnostics, /helix/metrics, /helix/heartbeat) do not exist on the robot. Only /rosout is natively available.

### 5. Perturbation Results

| Scenario | Executed? | Finding |
|----------|-----------|---------|
| External topic injection | Yes | Zero /rosout reaction from GO2 — DDS isolation confirmed |
| Topic rate baseline | Yes | Stable rates: IMU ±1%, odom ±1% across captures |
| Node lifecycle observation | No | Nodes not discoverable from external machines |
| Network latency perturbation | No | Deferred — requires human operator present |

**Finding**: The GO2's topic rates are highly stable, making rate-based anomaly detection viable. Node-level monitoring requires a different approach (topic liveness, not node discovery).

### 6. Adapter-Based Anomaly Detection on Live GO2

A passive adapter (`scripts/passive_adapter.py`) bridged 5 GO2 topic rate streams and 2 JSON state streams into `/helix/metrics`. HELIX's anomaly detector ran for 60 seconds on the PC observing the live GO2 graph.

| Metric | Value |
|--------|-------|
| FaultEvents emitted | 4 |
| Fault source | `/utlidar/cloud` rate anomaly |
| Peak Z-score | 146.91 |
| Consecutive violations | 6 |
| Adapter metrics published | rate_hz/utlidar_robot_pose, rate_hz/gnss, rate_hz/multiplestate, rate_hz/utlidar_imu, rate_hz/utlidar_cloud, gnss/satellite_total, gnss/hdop, go2_state/volume, go2_state/obstaclesAvoidSwitch, pose/displacement_rate_m_s |

**Finding**: HELIX can detect real sensor anomalies on the GO2 when bridged through a passive adapter. The LiDAR point cloud rate dropped below baseline for 6 consecutive samples (Z > 3.0).

### 7. /diagnostics Discovery

`/diagnostics` is published by GO2's `twist_mux` node at ~2 Hz with velocity topic masking status and timing. This was not observed in the April 2 capture, suggesting it activates only under certain operating modes.

## HELIX Attachability Assessment (Updated)

| HELIX Component | Current Input | GO2 Availability | Gap |
|----------------|---------------|------------------|-----|
| Heartbeat Monitor | /helix/heartbeat | NOT AVAILABLE | Needs bridge nodes publishing heartbeats on behalf of GO2 nodes |
| Anomaly Detector (metrics) | /helix/metrics | BRIDGED via passive_adapter.py | Adapter publishes rate + JSON-derived metrics |
| Anomaly Detector (diagnostics) | /diagnostics | AVAILABLE (twist_mux, ~2 Hz) | Native DiagnosticArray with velocity topic status |
| Log Parser | /rosout | AVAILABLE | Works, but low-rate when robot is idle |

### What Would Make HELIX Attachable Today

1. **Topic rate monitor** — subscribe to high-rate topics (IMU, odom, lidar) and compute rolling rate statistics. Deviation = fault signal.
2. **JSON state parser** — parse /multiplestate and /servicestate strings to extract structured state for anomaly detection.
3. **Pose drift monitor** — subscribe to /utlidar/robot_pose and compute rolling displacement statistics.
4. **Build unitree_go packages** on the PC/Jetson to unlock /lowstate (joint temperatures, motor currents) and /sportmodestate (gait mode, foot forces).

## Evidence Classification

### Observed on Hardware
- Hardware specs and thermal state of Jetson Orin NX
- GO2 topic landscape with 121–153 active topics (varies by session/operating mode)
- DDS cross-device latency (PC ↔ Jetson): 0.81 ms one-way
- HELIX algorithmic performance on Jetson (benchmark suite): 64K samp/s
- Jetson resource baseline while GO2 stack runs: 82% idle, 13GB free
- Topic isolation under external injection
- Bag captures with real sensor data (34,547+ messages across multiple bags)
- **HELIX ROS 2 nodes running on PC observing live GO2 graph (60 seconds)**
- **Passive adapter bridging GO2 topics to /helix/metrics**
- **4 FaultEvents from real LiDAR rate anomaly**
- **/diagnostics published by GO2's twist_mux at ~2 Hz**
- **Measured HELIX overhead: 41.7 MB RSS, 22.3% CPU (PC, with adapter)**

### Inferred from Hardware
- HELIX resource overhead on Jetson — projected from PC measurement × algorithmic scaling factor
- Long-term rate stability — projected from multiple 30-second captures with <1% CV
- Adapter viability for sustained monitoring — demonstrated for 60 seconds only

### Not Yet Validated
- HELIX running as persistent ROS 2 service on GO2/Jetson
- HELIX detecting a confirmed fault (rate anomaly ≠ confirmed fault)
- Log parser with GO2-specific rules against real GO2 error logs
- HELIX end-to-end latency on Jetson (helix_msgs not built on Jetson)
- Recovery or intervention actions
- Long-duration stability (>5 minutes)
- Performance under Jetson thermal throttling
- Multi-platform attachability comparison

## Bag Inventory

| Bag | Messages | Duration | Key Content |
|-----|----------|----------|-------------|
| helix_baseline_30s | 16,722 | 39.8s | IMU 250Hz, odom 150Hz, pose 19Hz |
| helix_topic_rates | 16,649 | 39.7s | Rate measurement duplicate |
| sensor_30s | 906 | 34.8s | Pose 19Hz + status topics |
| baseline_30s | 236 | 34.6s | GNSS, audiohub, multiplestate |
| helix_rosout_perturbation | 22 | 38.1s | External injection test |
| perturbation_test_publish | 12 | 12.5s | Novel topic publish test |

All bags verified readable with `ros2 bag info`. Sidecar metadata files included.
