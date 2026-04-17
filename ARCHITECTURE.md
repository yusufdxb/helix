# HELIX Sensing Architecture

This document describes the fault sensing architecture implemented in this repository. HELIX is a detection and reporting layer — it does not include recovery execution, diagnosis, or operator tooling.

## Sensing Scope

The sensing layer is organized around three questions:

1. **Is a node still alive?** — Heartbeat monitoring with configurable timeouts
2. **Are numeric metrics drifting?** — Rolling Z-score anomaly detection
3. **Are logs showing failure signatures?** — Regex-based log pattern matching

When any of these conditions is detected, the system publishes a structured `FaultEvent` message to `/helix/faults`.

## Packages

### `helix_msgs`

Custom ROS 2 message definitions:

- `FaultEvent.msg` — structured fault report (used by all three sensing nodes)
- `RecoveryHint.msg` — defined for future use; not referenced by any node in this codebase

### `helix_core`

Three lifecycle-managed sensing nodes:

- **`heartbeat_monitor.py`** — Subscribes to expected heartbeat topics. When a monitored node fails to publish within a configurable timeout for a threshold number of consecutive misses, emits a `FaultEvent`.

- **`anomaly_detector.py`** — Subscribes to `/helix/metrics`. Maintains a rolling window of recent samples and computes a Z-score for each incoming value. If the Z-score exceeds the configured threshold for a configurable number of consecutive samples, emits a `FaultEvent`.

- **`log_parser.py`** — Matches incoming log entries against a set of configured regex rules. When a pattern matches, emits a `FaultEvent` with the rule name and matched content.

### `helix_adapter`

Three lifecycle-managed adapter nodes that bridge non-standard robot topics into HELIX's `/helix/metrics` so the `helix_core` AnomalyDetector can run against a robot that does not natively publish HELIX-shaped inputs:

- **`topic_rate_monitor.py`** — Subscribes to a configured set of topics (default: GO2 IMU / odom / pose / cloud / GNSS / multiplestate), tracks callback-arrival timestamps in a rolling window, and publishes per-topic `rate_hz/<topic>` to `/helix/metrics`. Stale topics emit NaN.
- **`json_state_parser.py`** — Subscribes to `std_msgs/String` topics (default: `/gnss`, `/multiplestate`), parses JSON payloads, and republishes selected numeric and boolean fields as labeled metrics.
- **`pose_drift_monitor.py`** — Subscribes to a `geometry_msgs/PoseStamped` topic and publishes a rolling displacement-rate metric.

These replace the predecessor monolithic `passive_adapter.py` script (now archived under `hardware_eval_20260406/scripts/`); the canonical main-branch path is `ros2 launch helix_bringup helix_adapter.launch.py`.

### `helix_bringup`

Integration and demonstration:

- Launch file for the sensing stack (`helix_sensing.launch.py`) — auto-transitions the three `helix_core` lifecycle nodes through `configure → active`
- Launch file for the adapter stack (`helix_adapter.launch.py`) — auto-transitions the three `helix_adapter` lifecycle nodes
- YAML configuration for thresholds, timeouts, log rules, and adapter sources
- `fault_injector.py` (executable: `helix_fault_injector`) — publishes synthetic anomalies and stops heartbeats for local testing

## Data Flow

```text
/diagnostics ------------------------------+
                                           |
/helix/metrics ----------------------------+--> helix_core sensing nodes
                                           |         |
configured log rules / injected faults ----+         |
                                                     v
                                              /helix/faults (FaultEvent)
```

Each sensing node independently subscribes to its input source, processes data, and publishes `FaultEvent` messages to the shared `/helix/faults` topic. There is no inter-node coordination within the sensing layer.

## Design Choices

### Lifecycle nodes

All three monitors are implemented as ROS 2 lifecycle (managed) nodes. This allows controlled startup sequencing — nodes can be configured before activation and cleanly deactivated without killing the process. It also makes the sensing layer compatible with ROS 2 launch-time lifecycle management.

### Structured fault events

Faults are reported as typed ROS 2 messages (`FaultEvent`) rather than free-form log output. This makes downstream consumption deterministic — any future recovery layer, dashboard, or logging system can subscribe to `/helix/faults` without parsing strings.

### Conservative Z-score evaluation

The anomaly detector computes its Z-score against the existing history window **before** appending the new sample. This prevents the anomalous value from contaminating the baseline statistics it is being evaluated against. The detection is also gated by a `consecutive_trigger` parameter — a single outlier does not produce a fault event.

### Testable core logic

The sensing nodes are designed so their core detection logic can be exercised in unit tests via `rclpy` without requiring a full multi-node ROS 2 deployment.

## Proposed Extensions (Not Implemented)

The following are research directions documented for context. None are present in this codebase:

- Recovery planning and action execution
- Recovery verification loop (confirm that a recovery action resolved the fault)
- Operator-facing event dashboard
- Hardware deployment on the Unitree GO2 / Jetson Orin NX platform
- LLM-assisted diagnosis after deterministic detection
- Persistent event storage (SQLite or similar)

## Architecture Diagram Note

The diagram at `docs/images/architecture.svg` depicts the **target deployment context**, including the GO2 quadruped and Jetson Orin hardware. The sensing logic in this repository is platform-independent and has been validated through offline benchmarks, in-process ROS 2 unit tests, and four bounded hardware sessions on the live GO2 (longest run: 30 minutes on Jetson Orin NX). It has not been deployed as a persistent monitoring service. See `docs/GO2_HARDWARE_EVIDENCE.md` and `docs/LIMITATIONS.md` for the exact reproducibility boundary.
