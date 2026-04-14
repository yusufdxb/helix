# Paper Positioning

This document outlines viable framings for a workshop paper based on the HELIX repository, identifies the strongest option, and specifies claims that the current artifact does not support.

## 1. Viable Paper Framings

### Framing A (Recommended): Structured Fault Sensing for ROS 2

**Title direction**: "Structured fault sensing for ROS 2: a bounded prototype with offline evaluation"

This framing centers on the sensing architecture itself: the FaultEvent message abstraction, the three lifecycle-managed detection nodes, and the standalone benchmark methodology. It promises only what the repository delivers --- a prototype sensing layer with offline evaluation --- and avoids implying deployment, recovery, or hardware validation.

### Framing B: Design Lessons for Fault Observability

**Title direction**: "Toward self-monitoring ROS 2 robots: lessons from designing a fault observability layer"

This framing emphasizes the design decisions and trade-offs encountered when building a fault observability layer for ROS 2. It foregrounds architectural rationale (why lifecycle nodes, why Z-score before append, why a unified message type) and positions the work as a design study rather than a systems evaluation. Appropriate if the venue values experience reports.

### Framing C: Extending ROS 2 Diagnostics Infrastructure

**Title direction**: "Bridging ROS 2 diagnostics and fault-aware middleware: a prototype study"

This framing positions HELIX as an extension of the existing ROS 2 diagnostics stack, arguing that standard diagnostics lacks structured fault typing, heartbeat monitoring, and log-level pattern detection. Suitable if the target venue has a middleware or infrastructure focus, but requires careful scoping to avoid overstating the maturity of the prototype relative to the established diagnostics ecosystem.

## 2. Strongest Recommended Framing

**Framing A** is the strongest choice. It makes a bounded claim --- that a structured sensing prototype exists and has been evaluated offline --- and the repository fully supports that claim. The FaultEvent abstraction, the three detection nodes, and the benchmark results are all present and reproducible. This framing does not require hedging about missing components because it does not promise them.

## 3. What NOT to Claim

The following claims are **not supported** by the current repository and must not appear in the paper:

- **Hardware validation on GO2 or any physical robot.** The GO2 and Jetson Orin were physically accessible and ROS 2 topics were observed, but HELIX was never executed on the robot. Bag files captured from the GO2 contain standard ROS 2 topics, not HELIX fault data.
- **Recovery capability.** `RecoveryHint.msg` is defined but entirely unused. No recovery planning or execution logic exists.
- **LLM-assisted diagnosis.** No language model integration is present in the codebase.
- **Real-time performance.** The benchmarks evaluate pure-Python detection logic in a standalone process. They do not measure performance within a ROS 2 runtime, under real scheduling constraints, or on target hardware.
- **Deployment or field testing.** The system has not been deployed in any operational environment.
- **Real-world detection accuracy.** The reported TPR and FPR are computed on synthetic Gaussian data with injected anomalies. These results demonstrate that the detection logic functions correctly on well-behaved inputs; they do not constitute evidence of detection capability on real sensor data.

## 4. Likely Contribution Statement

> We present HELIX, a structured fault sensing prototype for ROS 2 that provides heartbeat monitoring, statistical anomaly detection, and log pattern matching through lifecycle-managed nodes and a unified FaultEvent message interface. We evaluate the detection components offline and discuss architectural decisions and hardware transfer challenges for the Unitree GO2 quadruped platform.
