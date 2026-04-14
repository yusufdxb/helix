# Contributions

Each contribution listed below is directly defensible from the repository contents.

1. **A structured FaultEvent message abstraction** that unifies crash detection, metric anomaly, and log pattern faults into a single typed ROS 2 interface. The `FaultEvent.msg` definition provides a common schema (fault type, severity, source node, description, timestamp) that downstream consumers can process without knowledge of which detector produced the event.

2. **Three composable, lifecycle-managed sensing nodes** with configurable parameters and conservative detection logic. The AnomalyDetector uses Z-score evaluation before appending new samples to the baseline window, preventing anomalous values from corrupting the reference distribution. The HeartbeatMonitor uses consecutive-miss thresholds to avoid false alarms from single dropped messages. The LogParser applies pre-defined regex rules with deduplication to suppress repeated alerts. All three nodes use ROS 2 lifecycle state management for orderly startup and shutdown.

3. **A standalone benchmark methodology** that evaluates detection latency, throughput, and TPR/FPR without requiring a ROS 2 runtime. The benchmark script extracts the core detection logic into pure-Python functions and measures performance against synthetic Gaussian data with known injected anomalies. This separation allows the detection algorithms to be profiled independently of ROS 2 scheduling and communication overhead.

4. **A fault injection demo node** for end-to-end verification of the sensing pipeline in simulation. The demo node publishes synthetic heartbeats, metric values, and log messages --- including deliberate anomalies --- to exercise all three detectors within a running ROS 2 graph.

5. **Documented architectural analysis of hardware transfer challenges** for deploying ROS 2 fault sensing on the Unitree GO2 quadruped with a Jetson Orin companion computer. This analysis identifies concrete obstacles (custom message types, topic landscape mismatch, resource constraints, DDS configuration) based on direct observation of the target platform.
