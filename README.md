# HELIX — Structured Fault Sensing for ROS 2 Systems

> A fault observability prototype: three lifecycle-managed detection nodes, a custom fault message type, and offline benchmarks.

[![CI](https://github.com/yusufdxb/helix/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/yusufdxb/helix/actions/workflows/ci.yml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-blue)](https://python.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## What This Is

HELIX is a bounded ROS 2 fault sensing prototype. It implements three lifecycle nodes that monitor a ROS 2 graph and publish structured `FaultEvent` messages when problems are detected:

- **Heartbeat monitor** — timeout-based liveness checks on expected nodes
- **Anomaly detector** — rolling Z-score over numeric metric streams
- **Log parser** — regex rule matching against log patterns

All three nodes publish to `/helix/faults` using a custom `FaultEvent.msg` type. A fault injector node is included for local demonstration and testing.

Offline benchmarks (pure-Python ports of the detection logic) are provided for evaluating algorithmic performance without a ROS 2 runtime.

## What This Is Not

This repository does **not** contain:

- A recovery or self-healing engine
- LLM-based diagnosis
- A web dashboard or operator UI
- Persistent event storage
- Hardware deployment artifacts or robot-specific code
- End-to-end integration with physical platforms

A `RecoveryHint.msg` is defined in `helix_msgs` but is not used by any node in this codebase.

## Architecture Overview

<p align="center">
  <img src="docs/images/architecture.svg" width="680"/>
</p>

```text
monitored ROS 2 graph
        |
        +--> /diagnostics -------------------+
        |                                    |
        +--> /helix/metrics -----------------+--> helix_core
        |                                    |    - heartbeat_monitor
        +--> log stream / fault rules -------+    - anomaly_detector
                                                  - log_parser
                                                  |
                                                  +--> /helix/faults (FaultEvent)
```

More detail: [ARCHITECTURE.md](ARCHITECTURE.md)

## Packages

| Package | Contents |
|---|---|
| `helix_msgs` | `FaultEvent.msg`, `RecoveryHint.msg` (defined but unused) |
| `helix_core` | `anomaly_detector.py`, `heartbeat_monitor.py`, `log_parser.py` |
| `helix_bringup` | Launch file, YAML config, `fault_injector.py` |

## Quick Start

### Build

```bash
mkdir -p ~/helix_ws/src
cd ~/helix_ws/src
git clone https://github.com/yusufdxb/helix.git
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch the sensing stack

```bash
ros2 launch helix_bringup helix_sensing.launch.py
```

### Inject faults (separate terminal)

```bash
ros2 run helix_bringup fault_injector
```

### Run benchmarks (no ROS required)

```bash
python3 benchmark_helix.py
```

## Evaluation

Five benchmark suites evaluate the sensing components:

| Benchmark | Key Result | ROS 2? |
|-----------|-----------|--------|
| Algorithmic throughput | ~331K samples/sec | No |
| End-to-end ROS 2 latency | 1.16 ms mean (p95: 1.24 ms) | Yes |
| Realistic anomaly detection | 96.5% TPR at Z=3.0 with marginal anomalies; 0% TPR for 3-sigma in Laplace noise | No |
| Log parser accuracy | 22/22 correct, ~754K msg/sec throughput | No |
| GO2 topic gap analysis | 1/4 HELIX inputs available on real hardware | No |

Full results, methodology, and caveats: [RESULTS.md](RESULTS.md)

## Testing

Unit tests exercise the three `helix_core` nodes via `rclpy` in isolation. ROS 2 Humble is required.

```bash
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select helix_core
colcon test-result --verbose
```

Full details: [TESTING.md](TESTING.md)

## Artifact Scope

An evaluator can reproduce the following locally:

1. **Build** — `colcon build` in a ROS 2 Humble environment
2. **Unit tests** — `colcon test --packages-select helix_core` (requires ROS 2 Humble)
3. **Standalone benchmarks** — `python3 benchmark_helix.py`, `python3 scripts/bench_realistic_anomalies.py`, `python3 scripts/bench_log_parser.py` (no ROS 2 required)
4. **End-to-end latency** — `python3 scripts/bench_e2e_latency.py` (requires ROS 2 Humble + built workspace)
5. **GO2 gap analysis** — `python3 scripts/go2_topic_gap_analysis.py` (no ROS 2 required)
6. **Live demo** — launch the sensing stack and inject faults in simulation

Steps 1, 2, 4, and 6 require ROS 2 Humble. The `ros:humble-ros-base` Docker image is a known-good environment. Steps 3 and 5 run with standard Python 3.10+. All result artifacts are stored in `results/`.

## Research Context

HELIX is designed with the Unitree GO2 quadruped and NVIDIA Jetson Orin Nano as a target deployment platform. The current codebase is platform-independent sensing logic validated offline and in simulation. No hardware-specific code or on-robot validation exists in this repository.

The architecture diagram (`docs/images/architecture.svg`) depicts the intended deployment context, including the target hardware — this reflects the research direction, not the current validated scope.

## Author

**Yusuf Guenena**
M.S. Robotics Engineering — Wayne State University
[linkedin.com/in/yusuf-guenena](https://linkedin.com/in/yusuf-guenena) · [github.com/yusufdxb](https://github.com/yusufdxb)
