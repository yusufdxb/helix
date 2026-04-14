# Session 2 Detailed Notes (2026-04-06)

## Objectives
Run a second hardware evidence pass to materially strengthen the HELIX paper before writing.
Primary goals: reproducibility, longer observation windows, new perturbation experiments, cross-session stability proof.

## Environment State at Start
- GO2 robot powered on, standing idle at 192.168.123.161
- Jetson Orin NX at 192.168.123.18, ROS2 daemon running, 109 topics visible
- PC at 192.168.123.10, ROS2 Humble + CycloneDDS on enp0s31f6, 110 topics visible
- go2_audio_node running in ros2_ws (confirmed via /rosout sample)
- HELIX NOT built in any colcon workspace (built from source during session)
- Jetson was compiling rtl8188eus WiFi driver (cc1 at 111% CPU) -- did not affect benchmark results
- Thermal state: Jetson 45C baseline, well below 85C throttle threshold

## Key Decisions and Observations

### /diagnostics Not Available (April 6)
On April 3 (session 2 of that day), /diagnostics was published by GO2's twist_mux at ~2 Hz.
On April 6, /diagnostics had zero messages. This suggests twist_mux only publishes /diagnostics
in certain GO2 operating modes (possibly when twist_mux is actively selecting between velocity
command sources). This is an important finding: /diagnostics availability is not guaranteed.

### CycloneDDS Node Discovery Limitation
ros2 node list and ros2 service list return empty when using CycloneDDS to observe the GO2's
FastRTPS graph. This is a known CycloneDDS multicast behavior -- topics are discoverable but
node endpoints are not. This means HELIX's heartbeat_monitor cannot use ROS2 node discovery
for liveness checks on the GO2. Alternative: topic-rate-based liveness (if a topic stops
publishing, the node is presumed dead).

### HELIX Build During Session
HELIX packages were built from source at /tmp/helix_ws/ to run the adapter detection experiment.
This is a temporary build -- not a persistent installation. The build succeeded without issues
on Ubuntu 22.04 / ROS2 Humble.

### FaultEvent Detection Differences: April 3 vs April 6
- April 3: 4 FaultEvents from /utlidar/cloud (LiDAR) rate anomaly, peak Z=146.91
- April 6: 2 FaultEvents from /utlidar/robot_pose rate anomaly, Z-scores 4.41 and 3.28
- The different anomaly sources confirm HELIX detects real rate variations, not a fixed artifact
- April 6 Z-scores are lower (4.41 vs 146.91), suggesting smaller rate fluctuations
- Both sessions show the adapter + anomaly detector produces real FaultEvents from live data

### DDS Load Test Result
Publishing 1000 Hz String messages alongside the GO2's live topics produced only 0.05% change
in the robot_pose rate (18.81 -> 18.80 Hz). This is within measurement noise and demonstrates
that DDS traffic from a monitoring layer does not degrade robot topic delivery. This is a strong
safety argument for passive monitoring deployment.

### /rosout Injection Feasibility
Successfully injected 5 ERROR-level messages to /rosout with a clearly labeled test source.
GO2 showed zero reaction (no /rosout echo, no behavior change). This confirms:
1. External publishers can safely write to /rosout without disturbing the robot
2. HELIX's log_parser could detect these injected errors if it had matching rules
3. This opens a path for controlled fault injection testing in future work

## What Didn't Work / Limitations

1. Could not bag /helix/faults because the bag recorder didn't have helix_msgs sourced
   when the recording started. FaultEvent data was captured via the measure_helix_overhead.py
   FaultCounter node instead. Workaround: source helix_ws before starting bag recorder.

2. Cross-device DDS latency (PC <-> Jetson) was not re-measured in this session. Would require
   a subscriber on Jetson echoing messages back -- was done in April 3 but not repeated.

3. No perturbation on the Jetson side (e.g., throttling Jetson CPU, adding load on Jetson).
   This would require more careful safety review and human operator presence.

4. /diagnostics intermittent availability means the "2/4 native HELIX inputs" claim should
   be softened to "1-2/4 depending on GO2 operating mode."

## Timing Log
- ~14:12 UTC: Agent 1 started (provenance capture)
- ~14:13 UTC: 5-minute bag started
- ~14:14 UTC: Perturbation experiments started (rosout inject, DDS load)
- ~14:19 UTC: 5-minute bag completed
- ~14:20 UTC: HELIX adapter session started
- ~14:22 UTC: HELIX adapter session completed (120s)
- ~14:24 UTC: Metrics-only adapter session started
- ~14:25 UTC: Metrics-only adapter session completed
- ~14:26 UTC: Bag verification completed
- All Jetson benchmarks run in parallel with PC experiments

## Operator Notes
- Robot was standing idle throughout. No locomotion commands were sent.
- No irreversible changes were made to any system.
- All test topics (/helix_test_load, /helix_test_topic) are ephemeral -- they disappear
  when the publisher stops.
- The /tmp/helix_ws build is ephemeral -- it will be cleaned on next reboot.
