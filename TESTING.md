# HELIX Testing

## Test Scope

The test suite consists mostly of **unit tests** that exercise individual
HELIX nodes and pure-logic modules via `rclpy` in isolation, plus one
`twist_mux` integration test that starts a real subprocess. Each unit test
instantiates a single node or pure function, feeds it synthetic inputs, and
verifies behaviour.

Apart from the `twist_mux` integration test, these are not multi-node or
end-to-end system tests. No actual robot hardware is involved.

### What Is Covered

- **Anomaly detection threshold behavior**: verifies that the Z-score detector fires on injected spikes and does not fire on normal data
- **Heartbeat timeout behavior**: verifies that missed heartbeats trigger fault events after the configured threshold
- **Log parsing and rule matching**: verifies that configured regex patterns match expected log entries and produce structured fault events
- **Adapter bridges**: rate-window, JSON state, and pose-drift monitors that feed `/helix/metrics`
- **Diagnosis tier**: the pure `rules` functions (R1-R4) and the `DiagnosisStateMachine` (IDLE / STOP_AND_HOLD transitions, RESUME clear window)
- **Recovery tier**: the pure `SafetyEnvelope` (enable flag, allowlist, per-fault cooldown, RESUME cooldown exemption) and `RecoveryNode` lifecycle / hint handling, including `RecoveryHint` consumption and zero-twist publication
- **Explanation tier**: the deterministic template formatter and the local-LLM client (HTTP layer mocked)

### What Is Not Covered

- Multi-node end-to-end integration beyond the single `twist_mux` subprocess test (nodes are otherwise tested individually, not as a connected graph)
- End-to-end pipeline latency (see [RESULTS.md](RESULTS.md) for standalone algorithmic benchmarks)
- Hardware-in-the-loop testing (no robot or embedded platform involved)
- A web dashboard (not implemented in this codebase)
- The LLM advisory path against a live `llama-server` (the client is unit-tested with the HTTP layer mocked; no test starts a real server)

## Test Files

`helix_core` lifecycle nodes:
- `src/helix_core/test/test_anomaly_detector.py`
- `src/helix_core/test/test_heartbeat_monitor.py`
- `src/helix_core/test/test_log_parser.py`

`helix_adapter` lifecycle nodes (rate, JSON, pose-drift bridges):
- `src/helix_adapter/test/test_rate_window.py`, `test_topic_rate_monitor.py`
- `src/helix_adapter/test/test_json_parse.py`, `test_json_state_parser.py`
- `src/helix_adapter/test/test_pose_drift.py`, `test_pose_drift_monitor.py`

`helix_diagnosis` (diagnosis tier):
- `src/helix_diagnosis/test/test_rules.py`, `test_diagnosis_node.py`, `test_context_buffer.py`

`helix_recovery` (recovery tier):
- `src/helix_recovery/test/test_recovery_node.py`

`helix_explanation` (explanation tier):
- `src/helix_explanation/test/test_llm_client.py`, `test_llm_explainer.py`

`helix_bringup` (twist_mux model + integration):
- `src/helix_bringup/test/test_twist_mux_model.py`, `test_twist_mux_integration.py`

Root-level repo-integrity (no ROS required):
- `tests/test_attachability.py`
- `tests/test_docs_consistency.py`
- `tests/test_adapter_migration.py`
- `tests/test_twist_mux_config.py`

## How to Run

```bash
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select helix_core helix_adapter
colcon test-result --verbose
```

Or with pytest directly (after `source install/setup.bash` for the sourced
suite; the root-level suite runs without ROS). The Python suite is 153 tests
across all packages as of 2026-05-29:

```bash
# Sourced ROS package tests: run one invocation per package so each gets a
# fresh rclpy context (mirrors CI):
for pkg in helix_core helix_adapter helix_diagnosis helix_recovery \
           helix_explanation helix_bringup; do
  python3 -m pytest "src/$pkg/test/" -q
done

# Root-level repo-integrity tests (no ROS required)
python3 -m pytest tests/ -q --ignore=tests/sim_integration
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
- No `colcon`-driven launch test asserts lifecycle activation under `ros2 launch`. The auto-activation contract is validated at the launch-file parse level in CI and by manual runs of `ros2 launch helix_bringup helix_sensing.launch.py` followed by `ros2 lifecycle get`.
- No tests exercise the fault injector script.
- Test coverage does not extend to message serialization or topic transport. Lifecycle transitions are exercised in `helix_recovery` (configure / activate / deactivate / cleanup) but not under sustained load.
