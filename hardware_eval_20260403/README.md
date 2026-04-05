# HELIX Hardware Evidence — 2026-04-03

Hardware evaluation evidence collected from a live Unitree GO2 quadruped robot,
Jetson Orin NX companion computer, and development PC.

## Hardware Topology

```
GO2 Robot (192.168.123.161)
   └── Ethernet ──→ Jetson Orin NX (192.168.123.18)
                        └── Ethernet ──→ PC (192.168.123.10)
```

- **GO2**: Unitree GO2 quadruped, 103 ROS 2 topics active
- **Jetson**: NVIDIA Jetson Orin NX, ARM Cortex-A78AE, 16GB RAM, Ubuntu 22.04, JetPack L4T R36.4.7
- **PC**: Intel i7-7700, 16GB RAM, Ubuntu 22.04, ROS 2 Humble

## Evidence Summary

### env/ — Hardware Provenance
| File | Content |
|------|---------|
| `jetson_system_info.txt` | Jetson hardware, OS, kernel, JetPack version |
| `jetson_ros2_topics.txt` | Full topic list with types (103 topics) |
| `jetson_ros2_nodes.txt` | Node list from Jetson |
| `jetson_topic_rates.txt` | Measured topic rates (IMU 250Hz, odom 150Hz, lidar 15Hz) |
| `jetson_tf_sample.txt` | TF availability check |
| `jetson_network.txt` | DDS/network configuration |
| `jetson_packages.txt` | GO2-related ROS 2 packages |
| `pc_system_info.txt` | PC hardware and OS info |
| `pc_ros2_env.txt` | PC ROS 2 environment |
| `helix_git_provenance.txt` | HELIX repo SHA and status |
| `provenance_summary.json` | Structured JSON of all provenance data |

### results/ — Benchmark Data
| File | Content |
|------|---------|
| `jetson_vs_pc_benchmarks.json` | Head-to-head benchmark comparison |
| `benchmark_comparison.json` | Detailed cross-platform results |
| `pc_benchmark_results.txt` | Raw PC benchmark output |
| `jetson_benchmark_results.txt` | Raw Jetson benchmark output |
| `jetson_resource_baseline.txt` | Jetson CPU/memory/thermal while GO2 runs |
| `cross_device_latency.txt` | PC↔Jetson DDS round-trip measurements |
| `resource_overhead_analysis.txt` | HELIX overhead analysis |

### bags/ — ROS 2 Bag Captures (6 bags)
| Bag | Messages | Duration | Key Topics |
|-----|----------|----------|------------|
| `helix_baseline_30s` | 16,722 | 39.8s | IMU, odom, robot_pose, status (from Jetson) |
| `helix_topic_rates` | 16,649 | 39.7s | Rate measurement capture (from Jetson) |
| `sensor_30s` | 906 | 34.8s | robot_pose (651@19Hz), gnss, status (from PC) |
| `baseline_30s` | 236 | 34.6s | gnss, audiohub, multiplestate (from PC) |
| `helix_rosout_perturbation` | 22 | 38.1s | Perturbation scenario A (from Jetson) |
| `perturbation_test_publish` | 12 | 12.5s | External topic injection test (from PC) |

All bags have metadata_sidecar.yaml files with provenance.

### notes/ — Experiment Protocols and Analysis
| File | Content |
|------|---------|
| `perturbation_feasibility.md` | What was/wasn't possible and why |
| `scenario_A_protocol.md` | /rosout observation under load |
| `scenario_B_protocol.md` | Node lifecycle observation (documented only) |
| `scenario_C_protocol.md` | Topic rate baseline measurement |
| `scenario_D_protocol.md` | Network latency perturbation (not executed) |

## Key Results

### Benchmark: Jetson vs PC
| Metric | PC | Jetson | Ratio |
|--------|-----|--------|-------|
| Anomaly detector throughput | 80,643 samp/s | 63,675 samp/s | 0.79x |
| Log parser throughput | 248,282 msg/s | 156,231 msg/s | 0.63x |
| Detection latency | 0.049 ms | 0.049 ms | 1.0x |
| Heartbeat miss latency | 200.7 ms | 200.7 ms | 1.0x |
| Accuracy | Identical | Identical | — |

### Cross-Device DDS Latency (PC ↔ Jetson)
| Metric | Value |
|--------|-------|
| RTT mean | 1.63 ms |
| One-way estimate | 0.81 ms |
| P95 RTT | 2.31 ms |
| Messages delivered | 50/50 (100%) |

### Jetson Resource State (GO2 stack running)
| Resource | Value |
|----------|-------|
| CPU idle | 82.4% |
| RAM used | 1.1 GB / 15.7 GB (7%) |
| Thermal | 43-48°C (safe) |
| Available cores | 4 of 8 |

### GO2 Topic Activity
| Topic | Rate | Type |
|-------|------|------|
| /utlidar/imu | 250 Hz | sensor_msgs/Imu |
| /utlidar/robot_odom | 150 Hz | nav_msgs/Odometry |
| /utlidar/robot_pose | 18.7 Hz | geometry_msgs/PoseStamped |
| /utlidar/cloud | 15.4 Hz | sensor_msgs/PointCloud2 |
| /gnss | 1 Hz | std_msgs/String (JSON) |
| /multiplestate | 1 Hz | std_msgs/String (JSON) |
| /audiohub/player/state | 4 Hz | std_msgs/String (JSON) |

### HELIX Input Availability
| HELIX Input | GO2 Status |
|-------------|-----------|
| /diagnostics | NOT published |
| /helix/heartbeat | NOT published |
| /helix/metrics | NOT published |
| /rosout | AVAILABLE (but low-rate when idle) |

## Proof Boundary

| Evidence Category | Status |
|-------------------|--------|
| Hardware access | CONFIRMED — SSH, ping, ROS 2 topic discovery all working |
| Passive observability | CONFIRMED — 103 topics visible, bags captured with real data |
| On-device performance | CONFIRMED — Benchmarks ran on Jetson with real numbers |
| Cross-device DDS latency | CONFIRMED — 50/50 RTT measurements |
| HELIX deployment on hardware | NOT DONE — HELIX nodes not running on GO2/Jetson |
| HELIX detecting real faults | NOT DONE — No fault events from real hardware |
| Active perturbation | PARTIAL — External topic injection tested; node-level perturbation not feasible safely |

## Reproducibility

All commands are documented in the artifact files. To reproduce:
1. Power on GO2 and Jetson with same network config
2. Run benchmark scripts from this repo on both machines
3. Record bags with the documented topic lists
4. Compare results against these artifacts
