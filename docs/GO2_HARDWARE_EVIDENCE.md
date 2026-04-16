# GO2 Hardware Evidence Summary

Evidence collected across two sessions (2026-04-03 and 2026-04-06) from a live Unitree GO2 quadruped, Jetson Orin NX companion, and development PC. Artifacts stored on T7 SSD at `hardware_eval_20260403/` and `hardware_eval_20260406/`.

**Session 1 (2026-04-03):** HELIX ROS 2 nodes ran on the live GO2 graph for the first time. A passive adapter bridging GO2 topic rates to `/helix/metrics` enabled the anomaly detector to emit 4 FaultEvents from a LiDAR rate fluctuation (unconfirmed as fault). `/diagnostics` was observed once with a `twist_mux` publisher present.

**Session 2 (2026-04-06):** Reproducibility validation after a 3-day gap. All Jetson benchmarks reproduced within 1% of Session 1 values. Adapter detection reproduced with 2 new FaultEvents from a `/utlidar/robot_pose` rate anomaly (peak Z-score 4.41) — a different source than Session 1's LiDAR fluctuation, consistent with HELIX reacting to real rate variation rather than a fixed artifact. An extended 5-minute bag capture collected ~104K messages with 100% in-window rate stability. Cross-session topic rate stability was strong (CV < 0.001 across the 3-day gap). A 1000 Hz DDS load test showed a 0.05% change in `/utlidar/robot_pose` rate (within measurement noise). `/diagnostics` was NOT observed in Session 2.

**Resolved /diagnostics status (Session 5, 2026-04-15 — see §9):** Across 6 sport-API states under `motion_switcher = normal`, `/diagnostics` had zero publishers (`ros2 topic info` returned `Unknown topic`). The Session 1 observation of `twist_mux` publishing `/diagnostics` is therefore **not reproducible** under default operating conditions and is treated as superseded. Native HELIX input coverage on the GO2 should be reported as **1/4** (`/rosout` only); the `2/4` figure that appears in earlier sections refers to the unreproduced Session 1 reading and is preserved verbatim only for traceability.

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
**Source:** `hardware_eval_20260403/results/cross_device_latency.txt` (no JSON artifact in `results/`).

| Metric | Value |
|--------|-------|
| RTT mean | 1.63 ms |
| One-way estimate | 0.81 ms |
| P95 RTT | 2.31 ms |
| Delivery | 50/50 (100%) |

**Finding**: Cross-device ROS 2 message transport adds <1 ms one-way latency. At HELIX's 10 Hz monitoring rate, this is 0.81% of the sensing period — negligible.

**Note:** This measurement is stored as a text log, not a structured JSON artifact. The numbers above are transcribed from that log. No independent replication artifact exists in `results/`.

### 3. Jetson Resource Headroom

Baseline measurement while GO2 stack (103 topics) is running, no HELIX.
**Source:** `hardware_eval_20260403/results/jetson_resource_baseline.txt` (no JSON artifact in `results/`).

| Resource | Value | HELIX Estimate |
|----------|-------|---------------|
| CPU idle | 82.4% | <0.5% of one core |
| RAM free | 13.0 GB / 15.7 GB | <50 MB additional |
| Thermal | 43–48°C (throttle at 85°C) | +0.2°C |
| Cores | 4 of 8 online | Needs 1 core |

**Finding**: The Jetson has substantial headroom. The HELIX resource estimates (<0.5% CPU, <50 MB RAM, +0.2°C) are **projected** from PC measurements and algorithmic scaling factors — HELIX ROS 2 nodes have not been measured running on the Jetson.

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

**Finding**: Rich sensor data available at standard ROS 2 types. The GO2 publishes sufficient data for meaningful HELIX monitoring, but HELIX's current input interfaces (/helix/metrics, /helix/heartbeat) do not exist on the robot. Of the four HELIX inputs, only `/rosout` is reliably available natively; `/diagnostics` was seen once in Session 1 from `twist_mux` (~2 Hz) but was confirmed absent across all 6 sport-API states tested in Session 5 under `motion_switcher = normal` — see §9.

### 5. Perturbation Results

**Note:** No structured JSON artifact exists for these observations. The bag files `helix_rosout_perturbation` and `perturbation_test_publish` provide partial evidence for the injection test. The remaining claims are observational notes without machine-readable backing.

| Scenario | Executed? | Finding | Evidence |
|----------|-----------|---------|----------|
| External topic injection | Yes | Zero /rosout reaction from GO2 — DDS isolation confirmed | Bag files in `hardware_eval_20260403/bags/` |
| Topic rate baseline | Yes | Stable rates: IMU ±1%, odom ±1% across captures | `results/bag_rate_analysis_cross_3bags.json` |
| Node lifecycle observation | No | Nodes not discoverable from external machines | — |
| Network latency perturbation | No | Deferred — requires human operator present | — |

**Finding**: The GO2's topic rates are highly stable, making rate-based anomaly detection viable. Node-level monitoring requires a different approach (topic liveness, not node discovery).

### 6. Adapter-Based Anomaly Detection on Live GO2

A passive adapter bridged 5 GO2 topic rate streams and 2 JSON state streams into `/helix/metrics`. HELIX's anomaly detector ran for 60 seconds on the PC observing the live GO2 graph. The Session 1 / Session 2 runs used the predecessor monolithic script `passive_adapter.py`, archived at `hardware_eval_20260406/scripts/passive_adapter.py`. The canonical main-branch path is now the `helix_adapter` ROS 2 package (lifecycle nodes `helix_topic_rate_monitor`, `helix_json_state_parser`, `helix_pose_drift_monitor`), launched via `ros2 launch helix_bringup helix_adapter.launch.py`.

| Metric | Value | Source |
|--------|-------|--------|
| FaultEvents emitted | 4 | `results/helix_overhead_with_adapter.json` |
| Fault source | `/utlidar/cloud` rate fluctuation | ibid. |
| Peak Z-score | 5.52 | ibid., first emitted event |
| Consecutive violations per event | 3 | ibid., each event states "3 consecutive samples" |
| Adapter metrics published | rate_hz/utlidar_robot_pose, rate_hz/gnss, rate_hz/multiplestate, rate_hz/utlidar_imu, rate_hz/utlidar_cloud, gnss/satellite_total, gnss/hdop, go2_state/volume, go2_state/obstaclesAvoidSwitch, pose/displacement_rate_m_s | — |

**Finding**: HELIX flagged a rate fluctuation in the LiDAR point cloud topic when bridged through a passive adapter. Each of the 4 emitted FaultEvents was triggered after 3 consecutive Z-score violations (Z > 3.0). Whether this fluctuation represents a genuine sensor anomaly or normal DDS transport jitter is unknown — no ground-truth labeling exists.

### 7. /diagnostics Discovery (Session 1 — superseded by §9)

`/diagnostics` was observed once in Session 1 (2026-04-03) being published by what appeared to be GO2's `twist_mux` node at ~2 Hz with velocity-topic masking status and timing. It was already absent in the April 2 pre-session capture and absent again in Session 2 (2026-04-06).

**Status:** Superseded by Session 5 (§9), which deliberately cycled the GO2 through 6 sport-API states under `motion_switcher = normal` and never observed a publisher on `/diagnostics`. The Session 1 sighting is treated as **non-reproducible under default operating conditions**. Treat the GO2's native HELIX-input coverage as **1/4** (`/rosout` only) unless and until `/diagnostics` is reproduced.

## HELIX Attachability Assessment (Updated)

| HELIX Component | Current Input | GO2 Availability | Gap |
|----------------|---------------|------------------|-----|
| Heartbeat Monitor | /helix/heartbeat | NOT AVAILABLE | Needs bridge nodes publishing heartbeats on behalf of GO2 nodes |
| Anomaly Detector (metrics) | /helix/metrics | BRIDGED via the `helix_adapter` package | Adapter publishes rate + JSON-derived + pose-drift metrics (Session 1/2 used the now-archived `passive_adapter.py`) |
| Anomaly Detector (diagnostics) | /diagnostics | NOT RELIABLY AVAILABLE (Session 5: 0 publishers across 6 sport-API modes; Session 1 sighting unreproduced) | Treat as unavailable for HELIX attachability accounting |
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
- **4 FaultEvents from LiDAR rate fluctuation (unconfirmed as fault)**
- **/diagnostics observed once in Session 1 from twist_mux at ~2 Hz (unreproduced in Sessions 2 and 5; treated as non-reliable)**
- **Measured HELIX overhead: 41.7 MB RSS, 22.3% CPU (PC, with adapter)**

### Inferred from Hardware
- HELIX resource overhead on Jetson — projected from PC measurement × algorithmic scaling factor
- Long-term rate stability — projected from multiple 30-second captures with <1% CV
- Adapter viability for sustained monitoring — demonstrated for 60 seconds only

### Not Yet Validated
- HELIX running as persistent ROS 2 service on GO2/Jetson
- HELIX detecting a confirmed fault (observed rate fluctuation ≠ confirmed fault; no ground truth)
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

---

## Session 5 update — 2026-04-15 (artifacts in `T7:LABWORK/HELIX/hardware_eval_20260415/`)

### 8. 30-minute persistent HELIX deployment on Jetson vs live GO2

All three lifecycle nodes ran on the Jetson Orin NX for 1780 s (target 1800 s)
against the live GO2 stack. Result: ALL `success_criteria` true
(`no_oom`, `no_thermal_throttle`, `lifecycle_active_throughout`).

| Metric | Heartbeat | Anomaly | Log Parser |
|--------|----------:|--------:|-----------:|
| RSS mean (MB) | 38.7 | 38.6 | 38.0 |
| RSS max (MB) | 41.3 | 41.0 | 39.3 |
| CPU mean (%) | 0.61 | 0.41 | 0.40 |
| `first_death_elapsed_s` | null | null | null |

Thermals: `cpu_max 54.97 °C, gpu_max 50.72 °C, soc_max 53.28 °C, tj_max 55.63 °C`,
no throttle observed across 1790 tegrastats samples.
Artifact: `results/jetson_persistent_30min_v3/jetson_persistent_30min.json`.

### 9. /diagnostics — re-verified absent under motion_switcher = normal

Cycled GO2 through 6 sport-API states under default motion_switcher mode.
Across all 6 captures (`baseline_idle`, `damp_on_standing`, `recovery_stand`,
`balance_stand_walk`, `obstacles_avoid_on`, `stand_down + damp`),
`ros2 topic info -v /diagnostics` returned **`Unknown topic`**, `Publisher count: 0`,
`ros2 topic hz` warned `does not appear to be published yet`. The
**Section-7 claim that twist_mux publishes /diagnostics is NOT reproduced**
under motion_switcher = normal. Modes `ai` and `advanced` were not tested
this session (operator-safety rationale); the twist_mux hypothesis remains
unverified for those modes. Artifact: `results/diagnostics_mode_matrix.json`.

### 10. Ground-truth fault injection on Jetson with HELIX deployed

12-min HELIX run on Jetson with 4 injection scenarios:

| # | Injection | Faults emitted | Detection latency | Match? |
|---|-----------|---------------:|-------------------|:------:|
| 1 | `/rosout` ERROR matching `nav2_costmap_fail` regex | **1** | **~1.8 s** | YES |
| 2 | LiDAR cover ~41 s (physical) | 0 | n/a | NO (gap) |
| 3 | 10000-msg flood on `/test/spam` | 0 | n/a | NO false-positive |
| 4 | USB mic disconnect ~35 s (physical) | 0 | n/a | NO (gap) |
| 5 | `kill -9` of GO2 node | INFEASIBLE (no SSH to MCU, GO2 nodes invisible to `ros2 node list`) | — | n/a |

**Headline:** end-to-end log-pattern detection works on hardware with low
latency. The 3 zero-fault outcomes for physical / network injections concretely
demonstrate the attachability gap predicted by `GO2_ATTACHABILITY_UPDATE.md`:
without an adapter bridging LiDAR rate / USB events / unrelated topics into
HELIX's 4 subscriptions, those signals are invisible.
Artifact: `results/ground_truth_injection.json`.

### 11. Operator-safety incident (documented for reproducibility)

A `Damp` (sport_api 1001) command issued during Task-5 mode cycling was
sent while the GO2 was standing — the script had read `sportmodestate.mode = 0`
and incorrectly inferred "robot is laying". `body_height` was the correct
pose indicator (≈ 0.32 m standing vs ≈ 0.07 m down). The robot collapsed
under released torque; no damage; recovered with `RecoveryStand` (1006).
Lessons recorded in `notes/incident_damp_collapse.md`.

### Updated "Observed on Hardware" additions (Session 5)

- HELIX 3-node lifecycle deployment on Jetson, 30 min, 0 deaths, success_criteria true
- Log-pattern fault end-to-end via `/rosout` → log_parser → `/helix/faults` (latency ~1.8 s)
- `/diagnostics` confirmed UNPUBLISHED across 6 sport-API states under motion_switcher=normal
- Hardware-grounded confirmation of LiDAR/USB attachability gap (3 physical injections, 0 detections)
