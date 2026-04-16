# Limitations

This document provides a candid accounting of the limitations of the HELIX prototype in its current state.

## Synthetic Benchmark Limits

The standalone benchmarks test pure-Python ports of the detection logic against synthetic Gaussian data with known injected anomalies. Real sensor data differs from this test regime in several ways: noise characteristics are non-Gaussian and sensor-specific; distributions may be multi-modal or non-stationary; failures can be correlated across multiple channels simultaneously; and transient environmental effects (vibration, thermal drift, communication dropouts) produce patterns not represented in the synthetic data. The reported detection latency (~0.065 ms), throughput (~46K samples/sec), and perfect TPR/FPR should be understood as measurements of algorithmic correctness on well-behaved inputs, not as predictions of operational performance.

## Bounded Hardware Validation

HELIX has been executed on real hardware in four bounded sessions; it has **not** been deployed as a persistent monitoring service on any robot. Specifically:

- **Session 1 (2026-04-03)** ran the three `helix_core` lifecycle nodes on a PC observing the live Unitree GO2 graph for 60 s, with a passive adapter bridging GO2 topics into `/helix/metrics`. Four FaultEvents were emitted from a `/utlidar/cloud` rate fluctuation; the fluctuation was not ground-truth labeled.
- **Session 2 (2026-04-06)** reproduced Session 1 after a 3-day gap (CV < 0.001 across the gap) and emitted 2 new FaultEvents from a `/utlidar/robot_pose` rate anomaly.
- **Session 5 (2026-04-15)** ran the three `helix_core` lifecycle nodes natively on the Jetson Orin NX against the live GO2 stack for 1780 s with no out-of-memory, no thermal throttle, and no lifecycle deaths. A controlled `/rosout` log-pattern injection produced an end-to-end FaultEvent in ~1.8 s.

What this is **not**: continuous deployment, ground-truth-labeled fault detection, multi-day stability, or operation under environmental disturbance. Three controlled physical injections (LiDAR cover, USB mic disconnect, `/test/spam` flood) produced 0 FaultEvents — those signals are invisible without an adapter explicitly bridging them. See `docs/GO2_HARDWARE_EVIDENCE.md` for full evidence, scope, and bag inventory. All artifacts referenced from there live under `hardware_eval_20260403/`, `hardware_eval_20260406/`, and `hardware_eval_20260415/` on the T7 evidence drive; the main branch ships the algorithm, the tests, and the documented launch paths, not the bag artifacts.

## No Passive or Non-Intrusive Integration

The `helix_core` sensing nodes are not passive observers. The HeartbeatMonitor requires monitored nodes to explicitly publish heartbeat messages on a designated topic. The AnomalyDetector requires metric values to be forwarded onto `/helix/metrics`. The `helix_adapter` package narrows but does not eliminate this gap: it bridges a fixed, configured set of GO2 topics (rate, JSON state, pose drift) into `/helix/metrics` so the AnomalyDetector can run against a robot that does not natively publish HELIX-shaped inputs. There is still no automatic node discovery and no introspection of arbitrary topics — adapter coverage is exactly what the YAML configures and nothing more. Session 5's three physical injections (LiDAR cover, USB mic disconnect, unrelated topic flood) produced 0 FaultEvents for this reason.

## No Verified Recovery

`RecoveryHint.msg` is defined in the message package but is not published, subscribed to, or processed anywhere in the codebase. No recovery planning, recovery execution, or fault response logic exists. The system detects and reports faults; it does not act on them.

## No End-to-End Field Deployment

The system has not been tested under real operational conditions. There is no evidence of sustained operation, no measurement of false alarm rates over extended periods, and no evaluation under the environmental disturbances (terrain variation, payload changes, weather, network instability) that characterize field robotics.

## No Multi-Robot or Distributed Testing

All testing assumes a single robot with co-located nodes. The architecture has not been evaluated in multi-robot scenarios, across network boundaries, or with the DDS discovery and QoS configurations required for distributed ROS 2 deployments.

## Detection Logic Limitations

- **AnomalyDetector**: The Z-score method assumes the baseline data is roughly Gaussian. For sensor channels with skewed, heavy-tailed, or multi-modal distributions, the Z-score threshold will produce systematically biased detection rates. No adaptive windowing, no distribution fitting, and no learning-based detection is implemented.
- **LogParser**: Detection depends entirely on pre-defined regex rules. Log messages that do not match any rule are silently ignored. There is no anomaly detection on log content, no frequency-based alerting, and no mechanism to learn new fault patterns from log data.
- **HeartbeatMonitor**: Uses a fixed timeout with a consecutive-miss threshold. There is no adaptive timeout based on observed communication latency, and no distinction between a node that has crashed and a node that is alive but slow.

## Threats to Validity

### Construct Validity

The benchmarks measure detection performance on synthetic Gaussian data with cleanly injected anomalies (values beyond a known threshold). This operationalization of "fault detection" does not capture the full complexity of real faults, which may manifest as subtle distributional shifts, correlated multi-channel deviations, or intermittent transient events. The construct measured (detection of point anomalies in Gaussian data) is a necessary but not sufficient proxy for the construct of interest (detection of real robot faults).

### Internal Validity

The standalone benchmarks isolate the detection logic from ROS 2 runtime effects (scheduling jitter, callback queuing, DDS serialization overhead, timer drift). Performance within a running ROS 2 graph may differ due to these factors. Additionally, the unit tests exercise individual nodes but do not test the full pipeline under sustained load or with realistic message rates.

### External Validity

The detection nodes have been tested against a narrow set of fault types: heartbeat timeout, Z-score threshold exceedance, and regex-matched log patterns. Generalization to fault types not represented in this set (hardware degradation, firmware errors, environmental interference, multi-node cascading failures) is not established. Similarly, the architecture has been developed and tested only within standard ROS 2 desktop environments; behavior on resource-constrained embedded platforms (Jetson Orin) and with non-standard DDS configurations is unknown.
