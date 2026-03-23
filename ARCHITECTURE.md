# HELIX Architecture

This document describes the architecture that is public in this repository today. It does not describe recovery, dashboard, or LLM subsystems as if they were already implemented.

## Public Scope

The current repo is a sensing layer for robotics faults in ROS 2. It is organized around three questions:
- Is a node still alive?
- Are numeric metrics drifting into abnormal ranges?
- Are logs showing known failure signatures?

When one of those conditions is met, the system emits a structured `FaultEvent`.

## Packages

### `helix_msgs`
Message package for communication between monitoring components and any future recovery layer.

Public messages now:
- `FaultEvent.msg`
- `RecoveryHint.msg`

### `helix_core`
Monitoring nodes.

Components now:
- `heartbeat_monitor.py`
  Monitors expected heartbeats and flags node liveness failures.
- `anomaly_detector.py`
  Consumes metric streams and computes rolling Z-scores before emitting anomaly events.
- `log_parser.py`
  Matches configured log rules and converts known signatures into structured faults.

### `helix_bringup`
Integration package.

Contents now:
- launch file for the sensing stack
- YAML configuration
- `fault_injector.py` for local demos and testing

## Data Flow

```text
/diagnostics ------------------------------+
                                           |
/helix/metrics ----------------------------+--> helix_core monitors --> /helix/faults
                                           |
configured log rules / injected faults ----+
```

The architecture is intentionally simple at this stage. The repo demonstrates detection and reporting, not autonomous recovery.

## Design Choices

### Lifecycle nodes
The monitors are built as lifecycle-aware ROS 2 nodes so they can be configured and activated in a controlled way.

### Structured events
The project uses ROS messages instead of free-form console output as the main fault interface. That keeps downstream recovery or operator tooling possible without re-parsing strings.

### Conservative statistics
`anomaly_detector.py` computes its Z-score against the existing history window before appending the newest value. That avoids contaminating the baseline with the anomaly it is trying to detect.

### Testable core logic
The current public code is most credible where it is bounded and testable. That is why the strongest part of the repo today is `helix_core`, not a broad unverified systems claim.

## Intended Future Expansion

These are design directions, not public deliverables in this repo today:
- recovery planning and action execution
- recovery verification loop
- operator-facing event dashboard
- hardware deployment on a real robot stack
- optional LLM-assisted diagnosis after deterministic detection
