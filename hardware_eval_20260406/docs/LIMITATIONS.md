# Limitations

This document provides a candid accounting of the limitations of the HELIX prototype in its current state.

## Synthetic Benchmark Limits

The standalone benchmarks test pure-Python ports of the detection logic against synthetic Gaussian data with known injected anomalies. Real sensor data differs from this test regime in several ways: noise characteristics are non-Gaussian and sensor-specific; distributions may be multi-modal or non-stationary; failures can be correlated across multiple channels simultaneously; and transient environmental effects (vibration, thermal drift, communication dropouts) produce patterns not represented in the synthetic data. The reported detection latency (~0.065 ms), throughput (~46K samples/sec), and perfect TPR/FPR should be understood as measurements of algorithmic correctness on well-behaved inputs, not as predictions of operational performance.

## Missing Hardware Validation

HELIX has never been executed on the Unitree GO2, Jetson Orin, or any physical robot. ROS 2 topics were observed on the GO2+Jetson platform, and small bag files were captured, but these contain standard ROS 2 messages (/joint_states, /odom, /rosout) --- not HELIX fault monitoring data. All integration testing has been performed in simulation and unit test environments on development hardware.

## No Passive or Non-Intrusive Integration

The sensing nodes are not passive observers. The HeartbeatMonitor requires monitored nodes to explicitly publish heartbeat messages on designated topics. The AnomalyDetector requires metric values to be forwarded into its subscription interface. There is no automatic node discovery, no introspection of arbitrary topics, and no mechanism to attach fault sensing to an existing ROS 2 graph without modifying the monitored nodes or adding bridge/adapter nodes.

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
