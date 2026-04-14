# Paper-Safe Claims, Threats to Validity, and Reviewer FAQ

## Abstract-Safe Claims

The following claims are defensible given current evidence:

> HELIX is a structured ROS 2 fault sensing prototype comprising three lifecycle-managed detection nodes. We evaluate its algorithmic performance on both a development PC and an NVIDIA Jetson Orin NX, demonstrating >600x throughput headroom relative to operational requirements. Through a hardware-grounded attachability study on a live Unitree GO2 quadruped, we characterize the gap between HELIX's monitoring interfaces and the GO2's topic landscape, and demonstrate that lightweight passive adapters can bridge standard ROS 2 topics to HELIX's input channels — enabling real-time anomaly detection on live robot sensor data without modifying the target platform.

**Claims you MUST NOT make:**
- "HELIX monitors the GO2" (it ran a 60-second evaluation, not continuous monitoring)
- "HELIX is deployed on the Jetson" (algorithmic benchmarks ran on Jetson; ROS 2 nodes ran on the PC observing GO2)
- "HELIX detected a fault on the GO2" (it detected a rate anomaly; whether that represents a real fault is unknown)
- "The adapter approach is validated" (one evaluation with 4 FaultEvents is a demonstration, not validation)

---

## Paper-Safe Claim Matrix

| Claim | Evidence | Strength | Safe Wording |
|-------|----------|----------|-------------|
| HELIX sensing logic is correct | Unit tests + benchmark suite | Strong | "The detection logic passes all unit tests and produces expected results across synthetic evaluation scenarios" |
| HELIX runs on Jetson hardware | benchmark_helix.py on Jetson | Strong | "HELIX's algorithmic core runs on the Jetson Orin NX at 64K samples/sec, providing 636x headroom over operational requirements" |
| HELIX can observe GO2 data | Adapter + 4 FaultEvents | Moderate | "A passive adapter bridging GO2 topic rates to HELIX's input channel enabled the anomaly detector to identify a real rate fluctuation in the LiDAR subsystem" |
| Cross-device latency is acceptable | 50 RTT measurements | Strong | "Cross-device DDS latency over wired Ethernet measured 0.81 ms one-way (50 trials), well within HELIX's 100 ms sensing period" |
| GO2 topic rates are stable | Bag analysis, 99.8% stability | Moderate | "Sensor topic rates showed >99% stability within 30-second windows and <1% coefficient of variation across captures" |
| Attachability is quantifiable | Matrix from 153 live topics | Novel | "We formalize 'attachability' as the fraction of a monitoring architecture's input interface natively satisfied by the target platform, measured at 50% for the GO2 with 54 additional topics accessible via standard-type adapters" |

---

## Threats to Validity

### Internal Validity

1. **Single evaluation session.** All hardware evidence was collected in a single session (2026-04-03). The 4 FaultEvents from the LiDAR rate anomaly may not be reproducible — the rate fluctuation could be a one-time DDS transport artifact, not a systematic LiDAR behavior.

2. **Synthetic baseline benchmarks.** The TPR/FPR sweep uses synthetic signals with extreme separability (spike = 1125 sigma). Real fault signals would have much smaller deviations. The realistic anomaly benchmark (Laplace noise, gradual drift) partially addresses this but uses generated data, not traces from real robot faults.

3. **Adapter as single-process proxy.** The passive adapter ran alongside HELIX in the same process group. In deployment, it would be a separate ROS 2 node with its own scheduling overhead. The measured resource overhead (22.3% CPU) conflates HELIX and adapter costs.

4. **Log parser with no GO2-specific rules.** The log parser had zero rules configured during hardware evaluation. No FaultEvents from log matching were demonstrated on real robot logs.

### External Validity

5. **Single robot platform.** Attachability was measured only on the Unitree GO2. Different ROS 2 robots may have higher or lower native coverage. The GO2 is a particularly challenging target due to its heavy use of custom message types.

6. **Wired same-subnet network.** Cross-device DDS latency was measured over a direct Ethernet connection (192.168.123.0/24). WiFi, multi-hop, or WAN scenarios would have different characteristics.

7. **Short observation windows.** Topic rate stability was measured over 30-second windows. Long-term drift, thermal effects, and operating mode transitions were not captured. The 5-minute bag (if completed) extends this but remains short relative to a real deployment.

8. **PC-side execution.** HELIX ROS 2 nodes ran on the PC (i7-7700), not on the Jetson Orin NX. Algorithmic benchmarks ran on both, but the full ROS 2 callback path was only measured on the PC. Resource overhead on the Jetson is projected, not measured.

### Construct Validity

9. **Rate anomaly ≠ fault.** The 4 FaultEvents detected a rate fluctuation in `/utlidar/cloud`. Without ground truth, we cannot confirm this represents an actual LiDAR fault vs. normal DDS transport jitter or GO2 operating mode change.

10. **Attachability as a metric.** The attachability matrix is a novel proposal. It has not been validated against other monitoring architectures or robot platforms. The 50% native coverage score reflects a specific definition of "attachable" that may not capture all relevant integration barriers (e.g., QoS compatibility, semantic correctness of adapted data).

---

## Reviewer FAQ

### "Did HELIX run on the GO2?"

HELIX's three ROS 2 lifecycle nodes (heartbeat monitor, anomaly detector, log parser) ran on a PC connected to the GO2's ROS 2 graph via wired Ethernet. The anomaly detector processed real GO2 sensor data through a passive adapter and produced 4 FaultEvents from a real LiDAR rate anomaly. The sensing logic (pure Python) was benchmarked directly on the Jetson Orin NX companion computer.

HELIX did **not** run as a persistent service on the robot. It was a 60-second controlled evaluation. The distinction matters: we demonstrated that HELIX can observe and react to GO2 data, not that it functions as a deployed monitoring system.

### "What exactly ran on the Jetson?"

Three things:
1. `benchmark_helix.py` — standalone anomaly detection and heartbeat monitoring benchmarks (pure Python, no ROS 2)
2. `bench_realistic_anomalies.py` — non-Gaussian noise, gradual drift, transient spike evaluation
3. `bench_log_parser.py` — log parser regex matching throughput

These are algorithmic benchmarks exercising the same detection logic used in the ROS 2 nodes but without ROS 2 runtime overhead. The results characterize compute cost of the detection algorithms on Jetson hardware. The full ROS 2 node stack (with callback scheduling, message serialization) was not run on the Jetson.

### "Why is /rosout not enough?"

`/rosout` carries log messages from all ROS 2 nodes. On the GO2, most nodes log at INFO level during normal operation; ERROR-level messages (which HELIX's log parser filters for) are rare during steady-state. In our 30-second captures, `/rosout` contained only recorder lifecycle messages — no GO2-specific error patterns.

For HELIX to detect faults via `/rosout`, the GO2 would need to be experiencing an error condition, and HELIX would need rules tuned to GO2-specific log patterns. Neither condition was met during evaluation. The log parser's *mechanism* works (22/22 accuracy on synthetic patterns), but its *rules* are not GO2-specific.

### "Why are 30-second windows insufficient?"

30-second windows demonstrate short-term rate stability (>99.8% for sensor topics) but cannot capture:
- Thermal throttling effects (Jetson Orin NX throttles at ~85°C; our measurement showed 43–48°C)
- Operating mode transitions (walk → stand → sit)
- Long-term sensor degradation
- DDS discovery churn under sustained operation

We mitigate this by: (a) cross-bag comparison showing consistent rates across independent captures, (b) the 5-minute bag extending the observation window, and (c) explicitly listing long-term stability as an unvalidated claim.

### "Why is the adapter contribution still interesting?"

The adapter approach addresses a real problem: most open-source ROS 2 monitoring assumes standard topics (`/diagnostics`, heartbeat protocols) that real robots don't publish. The GO2 has 153 topics, but only 2/4 map to HELIX's expected inputs.

Our contribution is:
1. **Quantifying the gap** — the attachability matrix measures exactly how much of a monitoring architecture's interface a platform natively satisfies (50% for GO2 + HELIX)
2. **Demonstrating the bridge** — a 200-line adapter node translates 5 rate streams and 2 JSON state streams into HELIX's metric input, enabling anomaly detection with zero GO2 modifications
3. **Observing real anomalies** — the adapter enabled detection of a real LiDAR rate fluctuation, demonstrating that the bridge produces actionable signals

The novelty is not the adapter code itself — it's the systematic analysis of monitoring architecture portability across non-standard robot platforms, with measured evidence.

---

## Evidence Classification Summary

### Observed on Hardware
- GO2 publishes 121–153 ROS 2 topics depending on operating mode
- `/diagnostics` published by twist_mux at ~2 Hz with velocity topic status
- `/rosout` available but low-rate during normal operation
- Sensor topic rates: robot_pose 18.8 Hz, gnss 1 Hz, multiplestate 1 Hz (99.8–100% stability)
- HELIX nodes ran for 60 seconds on the PC observing live GO2 graph
- Passive adapter bridged 5 rate + 2 JSON streams to /helix/metrics
- 4 FaultEvents from real /utlidar/cloud rate anomaly (Z=146.91 peak)
- HELIX algorithmic benchmarks on Jetson: 64K samp/s anomaly, 156K msg/s log parser
- Cross-device DDS RTT: 1.63 ms mean (PC ↔ Jetson, 50 trials)
- Jetson resource state: 82% idle CPU, 13GB free RAM with GO2 stack

### Inferred from Hardware
- HELIX full-stack overhead on Jetson (projected from PC measurement + algorithmic scaling)
- Long-term rate stability (projected from multiple short captures)
- Adapter viability for sustained monitoring (demonstrated for 60 seconds)
- Thermal headroom for HELIX on Jetson (43–48°C measured vs 85°C throttle point)

### Not Yet Validated
- HELIX running as persistent ROS 2 service on GO2/Jetson
- Fault detection accuracy against known GO2 fault scenarios
- Log parser with GO2-specific rules against real GO2 error logs
- Adapter output correctness (are derived metrics semantically meaningful?)
- Sustained operation under thermal stress
- Multi-robot or multi-platform attachability comparison
