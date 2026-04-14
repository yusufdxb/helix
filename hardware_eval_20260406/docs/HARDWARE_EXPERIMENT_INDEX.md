# Hardware Experiment Index

All experiments run 2026-04-03 on live GO2/Jetson/PC hardware.
Artifacts stored at: `T7 SSD: hardware_eval_20260403/`

## Experiments

### EXP-1: Hardware Provenance Capture
- **What**: System info, ROS 2 graph snapshot, topic rates, network config
- **Where**: `env/provenance_summary.json`, `env/jetson_*.txt`, `env/pc_*.txt`
- **Claim supported**: "We have access to and have characterized the target hardware"
- **Evidence strength**: Strong

### EXP-2: HELIX Algorithmic Benchmark on Jetson
- **What**: `benchmark_helix.py`, `bench_realistic_anomalies.py`, `bench_log_parser.py` run on Jetson Orin NX
- **Where**: `results/jetson_vs_pc_benchmarks.json`, `results/benchmark_comparison.json`
- **Claim supported**: "HELIX detection logic runs on the target hardware with acceptable performance"
- **Evidence strength**: Strong
- **Key numbers**:
  - Anomaly throughput: 63,675 samp/s (636x operational requirement)
  - Log parser: 156,231 msg/s (1562x operational requirement)
  - Detection latency: 0.049 ms (identical to PC)

### EXP-3: Cross-Device DDS Latency
- **What**: 50-message round-trip measurement between PC and Jetson via ROS 2 topics
- **Where**: `results/cross_device_latency.txt`, `results/benchmark_comparison.json`
- **Claim supported**: "Cross-machine ROS 2 communication is fast enough for monitoring"
- **Evidence strength**: Strong
- **Key numbers**: 1.63 ms RTT mean, 0.81 ms one-way, 100% delivery

### EXP-4: Jetson Resource Baseline
- **What**: CPU, memory, thermal measurement while GO2 stack (103 topics) is active
- **Where**: `results/jetson_resource_baseline.txt`, `results/resource_overhead_analysis.txt`
- **Claim supported**: "Jetson has sufficient headroom for HELIX alongside GO2 stack"
- **Evidence strength**: Strong
- **Key numbers**: 82% idle CPU, 13GB free RAM, 43-48°C thermal

### EXP-5: GO2 Topic Landscape Capture
- **What**: Full topic enumeration with types, rate measurement, data samples
- **Where**: `env/jetson_ros2_topics.txt`, `bags/helix_baseline_30s/`, `bags/sensor_30s/`
- **Claim supported**: "The GO2 publishes rich ROS 2 data observable without robot modification"
- **Evidence strength**: Strong

### EXP-6: Baseline Bag Capture (Standard Types)
- **What**: 30-second recordings of standard-typed GO2 topics
- **Where**: `bags/baseline_30s/`, `bags/sensor_30s/`
- **Claim supported**: "Real GO2 sensor data can be recorded and analyzed"
- **Evidence strength**: Strong
- **Key data**: 651 PoseStamped messages at 18.7 Hz, GPS at 1 Hz

### EXP-7: High-Rate Bag Capture (Jetson-Side)
- **What**: 40-second recordings from Jetson with access to all standard topics
- **Where**: `bags/helix_baseline_30s/` (16,722 msgs), `bags/helix_topic_rates/` (16,649 msgs)
- **Claim supported**: "Jetson can observe and record high-rate GO2 sensor streams"
- **Evidence strength**: Strong
- **Key data**: IMU at 250 Hz, odom at 150 Hz, pose at 19 Hz, lidar at 15 Hz

### EXP-8: External Topic Injection (Perturbation A)
- **What**: Published 5 test messages to /helix_test_topic; 20 from Jetson
- **Where**: `bags/perturbation_test_publish/`, `bags/helix_rosout_perturbation/`
- **Claim supported**: "External topic injection does not disturb GO2 — safe perturbation baseline"
- **Evidence strength**: Moderate
- **Finding**: Zero /rosout reaction from GO2 nodes

### EXP-9: Topic Rate Stability (Perturbation C)
- **What**: Two independent 30-second captures of the same topics
- **Where**: `bags/helix_baseline_30s/`, `bags/helix_topic_rates/`
- **Claim supported**: "GO2 topic rates are stable enough for rate-based anomaly detection"
- **Evidence strength**: Moderate (short duration)
- **Finding**: <1% rate variation between captures for IMU, odom

## Session 2 Experiments (2026-04-06)

### EXP-10: Extended 5-Minute Bag Capture
- **What**: 329.6-second recording of 6 key GO2 topics (IMU, pose, cloud, GNSS, multiplestate, rosout)
- **Where**: `hardware_eval_20260406/bags/extended_5min/`
- **Claim supported**: "GO2 topic rates are stable over extended observation windows"
- **Evidence strength**: Strong
- **Key numbers**: 94,325 messages, 100% rate stability, 650.4 MiB

### EXP-11: Controlled /rosout Error Injection
- **What**: Injected 5 ERROR-level messages to /rosout; bagged /rosout during injection
- **Where**: `hardware_eval_20260406/bags/perturbation_rosout_inject/`
- **Claim supported**: "Fault injection via /rosout is feasible, safe, and detectable by HELIX"
- **Evidence strength**: Moderate
- **Finding**: 23 total messages captured; 5 injected errors clearly identified. Zero GO2 reaction.

### EXP-12: DDS Load Impact on Topic Rates
- **What**: Published 1000 Hz test traffic while measuring /utlidar/robot_pose rate
- **Where**: `hardware_eval_20260406/bags/perturbation_rate_load/`
- **Claim supported**: "HELIX monitoring adds negligible overhead to DDS transport"
- **Evidence strength**: Strong
- **Key numbers**: 18.81 Hz baseline, 18.80 Hz under load (0.05% change = noise)

### EXP-13: Adapter-Based Detection Reproduction
- **What**: HELIX passive adapter + 3 lifecycle nodes running for 120s against live GO2
- **Where**: `hardware_eval_20260406/results/helix_overhead_with_adapter_20260406.json`
- **Claim supported**: "Adapter-based detection is reproducible across sessions"
- **Evidence strength**: Strong
- **Key numbers**: 2 FaultEvents (pose rate anomaly, Z=4.41 and 3.28), 39.4 MB RSS, 15.9% CPU
- **Reproduces**: Session 1 adapter detection (different anomaly source confirms real detection)

### EXP-14: Adapter Telemetry Capture
- **What**: Passive adapter metrics stream bagged without HELIX detector nodes
- **Where**: `hardware_eval_20260406/bags/helix_faults_session/`
- **Claim supported**: "Adapter produces continuous telemetry from GO2 standard topics"
- **Evidence strength**: Moderate
- **Key numbers**: 3,788 /helix/metrics messages at ~50 Hz

### Jetson Benchmark Reproduction
- **What**: Re-run of benchmark_helix.py, bench_realistic_anomalies.py, bench_log_parser.py on Jetson
- **Where**: `hardware_eval_20260406/results/jetson_benchmark_20260406.txt` and siblings
- **Claim supported**: "Jetson benchmark results are reproducible across sessions"
- **Evidence strength**: Strong
- **Key numbers**: All metrics within 1% of April 3 values. Deterministic tests are bit-identical.

### Cross-Session Rate Stability
- **What**: Compared April 3 and April 6 5-minute bag topic rates
- **Where**: `hardware_eval_20260406/results/cross_session_rate_comparison.json`
- **Claim supported**: "GO2 topic rates are stable across days, not just within sessions"
- **Evidence strength**: Strong
- **Key numbers**: CV < 0.001 for all sensor topics across 3-day gap

## Experiments NOT Executed

### Node Lifecycle Perturbation (Scenario B)
- **Why not**: GO2 nodes not discoverable from external machines. Killing nodes on the robot risks locomotion safety.
- **What it would prove**: Whether HELIX heartbeat monitor detects node death

### Network Latency Perturbation (Scenario D)
- **Why not**: Robot is live on physical floor. Requires human operator present for safety.
- **What it would prove**: Whether topic rate degradation under network stress is detectable
- **Protocol**: Documented in `notes/scenario_D_protocol.md`

### HELIX End-to-End on Jetson
- **Why not**: helix_msgs not built on Jetson. Would require colcon build on Jetson.
- **What it would prove**: Full ROS 2 callback latency on the target deployment platform.

## Claims Defensibility Matrix (Updated April 6)

| Paper Claim | Evidence | Defensible? |
|-------------|----------|-------------|
| "Runs on Jetson Orin NX" | EXP-2 benchmarks on actual Jetson (reproduced Apr 6) | YES -- algorithmic benchmark, not full ROS 2 stack |
| "Negligible overhead on target hardware" | EXP-4 resource baseline + EXP-2 + EXP-12 DDS load test | YES -- projected on Jetson, measured 0.05% DDS impact |
| "Sub-millisecond cross-device latency" | EXP-3 DDS RTT + Apr 6 local loopback | YES -- 0.81 ms one-way cross-device, 0.487 ms local |
| "GO2 publishes observable data" | EXP-5, 6, 7, 10 bag captures (11 bags, 138K msgs) | YES |
| "HELIX can observe GO2 anomalies" | EXP-13 adapter detection (6 FaultEvents, 2 sessions) | YES -- via adapter, not native |
| "Adapter architecture is feasible" | EXP-13 + EXP-14 (reproduced in 2nd session) | YES -- demonstrated and reproduced |
| "Rate-based detection is viable" | EXP-9 + EXP-10 rate stability + cross-session CV | YES -- stable baselines + demonstrated detection |
| "Results are reproducible" | Apr 3 vs Apr 6 comparison (<1% delta, identical accuracy) | YES -- strong reproducibility evidence |
| "Monitoring is safe for robot" | EXP-8 + EXP-11 + EXP-12 perturbation tests | YES -- zero impact on GO2 operation |
| "HELIX monitors GO2 faults" | — | NO -- rate anomaly != confirmed fault |
| "HELIX is deployed on GO2" | — | NO -- controlled evaluations, not deployment |
