# GO2 Hardware Lessons

This document describes concrete reasons the current HELIX design does not transfer cleanly to the Unitree GO2 quadruped platform with a Jetson Orin companion computer. These observations are based on direct inspection of the GO2's ROS 2 topic landscape and development experience with the Jetson Orin.

## 1. Custom Message Types

The GO2 uses `unitree_go` and `go2_interfaces` message packages that are not part of the standard ROS 2 distribution. These packages define proprietary message types for motor state, body pose, sport mode commands, and low-level control interfaces. HELIX subscribes to standard ROS 2 topics (`/diagnostics`, `/rosout`, generic metric topics) which may not carry GO2-specific fault signals. Without the Unitree message definitions installed and integrated, HELIX cannot parse or monitor the robot's native state reporting.

## 2. Topic Landscape Mismatch

The GO2 publishes over 122 topics spanning motor control, IMU data, body state, camera streams, sport mode status, and proprietary system topics. HELIX monitors three topic types: heartbeat, numeric metric, and log messages. Mapping the GO2's proprietary state data into HELIX's metric/heartbeat/log model requires significant adaptation work --- specifically, deciding which of the 122+ topics carry fault-relevant information, what constitutes a "normal" baseline for each, and how to convert proprietary message fields into the generic numeric values that the AnomalyDetector expects.

## 3. Jetson Resource Constraints

The Jetson Orin NX has limited CPU cores and memory compared to a development workstation. The overhead introduced by HELIX's lifecycle nodes, timer callbacks (heartbeat checks, metric polling), and Z-score windowing (maintaining and computing statistics over rolling windows) has not been profiled on-device. On a platform that is simultaneously running locomotion control, perception, and communication stacks, additional ROS 2 nodes competing for CPU time and memory could degrade overall system performance. Profiling and resource budgeting on the target hardware are prerequisites for deployment.

## 4. DDS Configuration

The current repository assumes loopback (single-machine) ROS 2 communication. A real GO2 deployment involves multi-device networking: the GO2's onboard computer communicates with the Jetson Orin over a local network. This requires DDS discovery configuration (domain IDs, discovery servers or multicast settings), QoS policy tuning for reliability and latency over a real network link, and potentially DDS security configuration. None of this is present in the repository.

## 5. Missing Hardware Fault Types

The GO2 can experience motor overtemperature, IMU drift and calibration loss, battery voltage drops and charging faults, actuator position errors, and gait controller failures. HELIX has no log regex rules for any of these fault types, no metric channels configured for motor temperature or battery voltage, and no heartbeat expectations for GO2-specific control nodes. The detection logic is generic; the fault knowledge required to apply it to the GO2 does not exist in the current configuration.

## 6. No SDK Integration

Unitree provides an SDK for low-level control and state reporting on the GO2. This SDK exposes motor-level telemetry, safety limits, and control mode transitions that are not available through standard ROS 2 topics. Integrating HELIX with the Unitree SDK would provide access to fault-relevant data that the current ROS 2-only approach cannot observe, but this integration has not been implemented.

## 7. Architectural Changes Needed

Transferring HELIX to the GO2+Jetson platform would require at minimum:

- **Adapter nodes** to bridge GO2 proprietary topics (motor state, body pose, sport mode status) into HELIX's expected metric and heartbeat interfaces.
- **Hardware-specific log rules** for GO2 fault patterns, derived from documentation review and on-device log analysis.
- **On-device performance profiling** to measure lifecycle node overhead, callback latency, and memory usage on the Jetson Orin under realistic concurrent workloads.
- **Graceful degradation under thermal throttling**, since the Jetson Orin reduces clock speeds under sustained thermal load, which could affect timer accuracy and detection latency.
- **DDS network configuration** for reliable multi-device communication between the GO2 and the Jetson.
- **Unitree SDK integration** for access to low-level motor and safety telemetry not exposed through ROS 2 topics.

These changes represent a substantial engineering effort beyond the current prototype scope.
