# HELIX — Hierarchical Execution and Lifecycle Intelligence eXecutor

Self-healing middleware for ROS 2 robotic systems. Detects node failures,
sensor anomalies, and log-pattern faults autonomously. Built for the
Unitree GO2 on ROS 2 Humble.

## Architecture
- Phase 1: Fault sensing (heartbeat monitor, anomaly detector, log parser)
- Phase 2: Recovery engine (rule-based + adaptive)
- Phase 3: On-device LLM advisor (Ollama / Phi-3 Mini on Jetson)
- Phase 4: Web dashboard

## Requirements
- ROS 2 Humble
- Ubuntu 22.04
- Python 3.10+

## Build
    cd helix_ws
    colcon build --symlink-install
    source install/setup.bash

## Launch
    ros2 launch helix_bringup helix_sensing.launch.py
