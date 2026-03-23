# HELIX

> ROS 2 fault-sensing prototype for robotics systems.

**Status:** public Phase 1 prototype  
**Current scope:** fault detection, fault injection, and ROS 2 message plumbing  
**Not yet public in this repo:** automated recovery, LLM diagnosis, dashboard, or hardware deployment

[![CI](https://github.com/yusufdxb/helix/actions/workflows/ci.yml/badge.svg?branch=feat/phase2-recovery)](https://github.com/yusufdxb/helix/actions/workflows/ci.yml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-blue)](https://python.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

> **Development branches:**
> [`feat/phase2-recovery`](https://github.com/yusufdxb/helix/tree/feat/phase2-recovery) — recovery engine, CI, benchmark ([PR #1](https://github.com/yusufdxb/helix/pull/1)) ·
> [`feat/phase3-llm`](https://github.com/yusufdxb/helix/tree/feat/phase3-llm) — Ollama/Phi-3 integration ·
> [`feat/phase4-dashboard`](https://github.com/yusufdxb/helix/tree/feat/phase4-dashboard) — web dashboard

## What This Repo Actually Contains

HELIX is an experiment in monitoring ROS 2 systems for early signs of failure. The current public repository is focused on sensing and reporting faults, not on full self-healing autonomy.

Implemented now:
- `helix_core` lifecycle nodes for heartbeat monitoring, anomaly detection, and log parsing
- `helix_msgs` custom messages for publishing fault events and recovery hints
- `helix_bringup` launch/config package plus a fault injector for local demos
- unit tests for the `helix_core` sensing components
- architecture diagrams for the intended system direction

Not implemented in this public tree:
- recovery execution engine
- LLM-backed diagnosis
- web dashboard
- persistent SQLite event store
- Jetson or GO2 deployment artifacts

That distinction matters. This repo should be read as a solid Phase 1 sensing prototype, not a finished self-healing robotics middleware stack.

## Why It Is Still Worth Looking At

The project targets a real robotics problem: failures are often detectable before they become mission-ending. Even at this prototype stage, the repo shows useful systems thinking:
- lifecycle-node based monitoring
- structured `FaultEvent` messages instead of ad hoc log strings
- multiple sensing paths: heartbeats, numeric anomalies, and log pattern rules
- offline tests around the core monitoring logic

## Current Architecture

```text
monitored ROS 2 graph
        |
        +--> /diagnostics -------------------+
        |                                    |
        +--> /helix/metrics -----------------+--> helix_core
        |                                    |    - heartbeat_monitor
        +--> log stream / fault rules -------+    - anomaly_detector
                                             |    - log_parser
                                             |
                                             +--> /helix/faults (FaultEvent)
                                             +--> /helix/recovery_hints

helix_bringup
- launch files
- YAML config
- fault_injector demo node
```

More detail: [ARCHITECTURE.md](ARCHITECTURE.md)

## Packages

| Package | What is in the repo now |
|---|---|
| `helix_msgs` | `FaultEvent.msg`, `RecoveryHint.msg` |
| `helix_core` | `anomaly_detector.py`, `heartbeat_monitor.py`, `log_parser.py` |
| `helix_bringup` | launch file, config, and `fault_injector.py` |

## Build

```bash
mkdir -p ~/helix_ws/src
cd ~/helix_ws/src
git clone https://github.com/yusufdxb/helix.git
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch helix_bringup helix_sensing.launch.py
```

In a separate terminal:

```bash
ros2 run helix_bringup fault_injector
```

The launch path above reflects the files that currently exist in the repository.

## Fault Types in the Public Prototype

| Source | Mechanism | Output |
|---|---|---|
| Heartbeat monitoring | timeout-based liveness checks | `FaultEvent` |
| Metric anomaly detection | rolling Z-score over recent samples | `FaultEvent` |
| Log parsing | regex and rule-based matching | `FaultEvent` |

## Testing

`helix_core` includes local tests for the current sensing logic.

```bash
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select helix_core
colcon test-result --verbose
```

More detail: [TESTING.md](TESTING.md)

## Roadmap

The intended direction is still useful, but it is future scope, not current scope:
- rule-based recovery actions after fault detection
- verification loop for recovery success/failure
- hardware validation on a real robot stack
- optional operator dashboard
- optional LLM-assisted diagnosis

## Author

**Yusuf Guenena**  
M.S. Robotics Engineering — Wayne State University  
[linkedin.com/in/yusuf-guenena](https://linkedin.com/in/yusuf-guenena) · [github.com/yusufdxb](https://github.com/yusufdxb)
