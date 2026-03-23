# HELIX Testing

This repository currently has meaningful test coverage only for the public sensing layer in `helix_core`.

## What Is Tested

The current test suite targets the bounded logic that is actually in the repo:
- anomaly detection threshold behavior
- heartbeat timeout behavior
- log parsing and rule matching

The tests are in:
- `src/helix_core/test/test_anomaly_detector.py`
- `src/helix_core/test/test_heartbeat_monitor.py`
- `src/helix_core/test/test_log_parser.py`

## How To Run

```bash
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select helix_core
colcon test-result --verbose
```

## What Is Not Yet Covered

The repo does not currently include public tests for:
- recovery execution, because that subsystem is not in this tree
- dashboard behavior, because that subsystem is not in this tree
- LLM diagnosis, because that subsystem is not in this tree
- hardware deployment, because the repo is still a local prototype

## Employer-Relevant Reading

The presence of tests here is useful because it shows the most mature part of the project is the part that can actually be validated today: the sensing logic. That is a stronger signal than broad claims around unreleased subsystems.
