# HELIX Testing

## Test Scope

The test suite consists of **unit tests** that exercise the three `helix_core` sensing nodes via `rclpy` in isolation. Each test instantiates a single node, feeds it synthetic inputs, and verifies that fault detection behaves correctly.

These are not integration tests, multi-node tests, or end-to-end system tests. No actual robot hardware or multi-process ROS 2 graphs are involved.

### What Is Covered

- **Anomaly detection threshold behavior** — verifies that the Z-score detector fires on injected spikes and does not fire on normal data
- **Heartbeat timeout behavior** — verifies that missed heartbeats trigger fault events after the configured threshold
- **Log parsing and rule matching** — verifies that configured regex patterns match expected log entries and produce structured fault events

### What Is Not Covered

- Recovery execution (not implemented in this codebase)
- Multi-node integration (nodes are tested individually, not as a connected graph)
- End-to-end pipeline latency (see [RESULTS.md](RESULTS.md) for standalone algorithmic benchmarks)
- Hardware-in-the-loop testing (no robot or embedded platform involved)
- Dashboard or LLM subsystems (not implemented in this codebase)
- `RecoveryHint.msg` publishing (message is defined but unused)

## Test Files

- `src/helix_core/test/test_anomaly_detector.py`
- `src/helix_core/test/test_heartbeat_monitor.py`
- `src/helix_core/test/test_log_parser.py`

## How to Run

```bash
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select helix_core
colcon test-result --verbose
```

Or with pytest directly:

```bash
cd src/helix_core
python3 -m pytest test/ -v
```

## Reproducibility

### Environment Requirements

- **ROS 2 Humble** with `rclpy` (required for all tests)
- **Python 3.10+**
- The `helix_msgs` package must be built first (`colcon build` builds all packages in dependency order)

### Known-Good Environment

Tests are validated in the `ros:humble-ros-base` Docker container. To reproduce in Docker:

```bash
docker run -it --rm -v $(pwd):/ws ros:humble-ros-base bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --packages-select helix_core
colcon test-result --verbose
```

### Continuous Integration

CI runs these tests automatically on every push to `main` via the GitHub Actions workflow at `.github/workflows/ci.yml`. The CI environment uses the same `ros:humble-ros-base` image.

## Limitations

- Tests depend on `rclpy` and cannot run without a ROS 2 installation. The standalone benchmark (`benchmark_helix.py`) provides a ROS-free way to evaluate detection logic.
- No integration tests verify the behavior of multiple sensing nodes running together in a single ROS 2 graph.
- No tests exercise the fault injector or launch file.
- Test coverage does not extend to message serialization, topic transport, or lifecycle state transitions under load.
