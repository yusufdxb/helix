# HELIX — Hierarchical Execution and Lifecycle Intelligence eXecutor

Self-healing middleware for ROS 2 robotic systems. Detects node failures,
sensor anomalies, and log-pattern faults autonomously — then executes
recovery actions and verifies they worked. Built for the Unitree GO2 on
ROS 2 Humble.

## Status

| Phase | Description | Status |
|-------|-------------|--------|
| Phase 1 | Fault sensing (heartbeat monitor, anomaly detector, log parser) | Complete |
| Phase 2 | Recovery engine (rule-based + adaptive, verification loop) | Complete |
| Phase 3 | On-device LLM advisor (Ollama / Phi-3 Mini on Jetson Orin) | Planned |
| Phase 4 | Web dashboard | Planned |

## Architecture

```
helix_core/
  heartbeat_monitor.py   — watches node liveness via topic activity
  anomaly_detector.py    — Z-score statistical sensor anomaly detection
  log_parser.py          — regex pattern matching on rosout log stream

helix_recovery/
  recovery_planner.py    — rule-based fault → action mapping
  action_executor.py     — executes recovery actions (restart, reroute, etc.)
  verification_loop.py   — confirms recovery succeeded before clearing fault
  state_db.py            — persistent fault/recovery history (SQLite)

helix_msgs/              — custom ROS 2 message types (FaultEvent, RecoveryAction)
helix_llm/               — Phase 3: Ollama/Phi-3 Mini advisor (not yet active)
helix_dashboard/         — Phase 4: web dashboard (not yet active)
helix_bringup/           — launch files and lifecycle orchestration
```

### Fault Detection Pipeline

```
Sensor topics  ──► anomaly_detector  ──► /helix/anomaly_events
rosout logs    ──► log_parser        ──► /helix/log_events      ──► recovery_planner ──► action_executor ──► verification_loop
Node activity  ──► heartbeat_monitor ──► /helix/heartbeat_events
```

## Requirements

- ROS 2 Humble
- Ubuntu 22.04
- Python 3.10+
- CycloneDDS

## Environment Setup

```bash
source ~/helix_ws/helix_env.sh
```

Sets `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=1`, and `CYCLONEDDS_URI` for
reliable single-machine DDS discovery. Lifecycle transitions are managed
programmatically via `rclpy` — no CLI overhead or DDS discovery races.

## Build

```bash
cd ~/helix_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
# Full sensing + recovery pipeline
ros2 launch helix_bringup helix_sensing.launch.py

# Individual nodes (development)
ros2 run helix_core heartbeat_monitor
ros2 run helix_core anomaly_detector
ros2 run helix_recovery recovery_planner
```

## Testing

```bash
cd ~/helix_ws/src/<package>/
python3 -m pytest test/ -v
```

## Key Design Decisions

- **Programmatic lifecycle management** — all lifecycle transitions use the
  `rclpy` lifecycle manager directly; no `ros2 lifecycle` CLI calls which cause
  DDS discovery races during startup.
- **`ROS_LOCALHOST_ONLY=1`** instead of a custom CycloneDDS XML — simpler and
  more reliable for single-host development; avoids shared-memory transport issues.
- **SQLite state_db** — persists fault/recovery history across restarts so the
  recovery planner can detect recurring faults and escalate its strategy.
