# Paper-Safe Claims, Threats to Validity, and Reviewer FAQ

## Abstract-Safe Claims

The following claims are defensible given current evidence:

> HELIX is a structured ROS 2 fault sensing prototype comprising three lifecycle-managed detection nodes. We evaluate its algorithmic performance on both a development PC and an NVIDIA Jetson Orin NX, demonstrating >600x throughput headroom relative to operational requirements. Through a hardware-grounded attachability study on a live Unitree GO2 quadruped, we characterize the gap between HELIX's monitoring interfaces and the GO2's topic landscape, and demonstrate that lightweight passive adapters can bridge standard ROS 2 topics to HELIX's input channels — enabling a 60-second demonstration of anomaly detection on live robot sensor data without modifying the target platform.

**Claims you MUST NOT make:**
- "HELIX monitors the GO2" (it ran a 60-second evaluation, not continuous monitoring)
- "HELIX is deployed on the Jetson" (algorithmic benchmarks ran on Jetson; ROS 2 nodes ran on the PC observing GO2)
- "HELIX detected a fault on the GO2" (it flagged a rate fluctuation; whether that represents a real fault is unknown — no ground truth)
- "The adapter approach is validated" (one 60-second evaluation with 4 FaultEvents is a demonstration, not validation)

---

## Paper-Safe Claim Matrix

| Claim | Evidence | Strength | Safe Wording |
|-------|----------|----------|-------------|
| HELIX sensing logic is correct | Benchmark suite (`results/realistic_anomaly_results.json`, `results/e2e_latency_results.json`). Unit test results: **no artifact in `results/`** — unit tests require ROS 2 Humble and are not included in offline validation. | Moderate | "The detection logic produces expected results across synthetic evaluation scenarios. Unit test results are not captured as a stored artifact." |
| HELIX runs on Jetson hardware | benchmark_helix.py on Jetson | Strong | "HELIX's algorithmic core runs on the Jetson Orin NX at 64K samples/sec, providing 636x headroom over operational requirements" |
| HELIX can observe GO2 data | Adapter + 4 FaultEvents (`results/helix_overhead_with_adapter.json`) | Moderate | "A passive adapter bridging GO2 topic rates to HELIX's input channel enabled the anomaly detector to flag a rate fluctuation in the LiDAR subsystem (peak Z=5.52, 3 consecutive violations per event). Whether this represents a genuine fault is unknown." |
| Cross-device latency is acceptable | 50 RTT measurements (`hardware_eval_20260403/results/cross_device_latency.txt` — text log, no JSON in `results/`) | Moderate | "Cross-device DDS latency over wired Ethernet measured 0.81 ms one-way (50 trials), well within HELIX's 100 ms sensing period. **Unsupported by structured artifact in `results/`; evidence is a text log in `hardware_eval_20260403/`.**" |
| GO2 topic rates are stable | Bag analysis, 99.8% stability | Moderate | "Sensor topic rates showed >99% stability within 30-second windows and <1% coefficient of variation across captures" |
| Attachability is quantifiable | Matrix from 153 live topics (`results/attachability_matrix.json`) | Novel | "We formalize 'attachability' as the fraction of a monitoring architecture's input interface natively satisfied by the target platform, measured at 50% for the GO2 with 54 additional topics identified as adapter candidates (24 demonstrated, 30 heuristic String-type candidates)" |

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

HELIX's three ROS 2 lifecycle nodes (heartbeat monitor, anomaly detector, log parser) ran on a PC connected to the GO2's ROS 2 graph via wired Ethernet. The anomaly detector processed live GO2 sensor data through a passive adapter and produced 4 FaultEvents from an observed LiDAR rate fluctuation (peak Z=5.52). The sensing logic (pure Python) was benchmarked directly on the Jetson Orin NX companion computer.

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
3. **Observing rate fluctuations** — the adapter enabled detection of a LiDAR rate fluctuation (not confirmed as a fault), demonstrating that the bridge produces signals the detector can act on

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
- 4 FaultEvents from /utlidar/cloud rate fluctuation (Z=5.52 peak, 3 consecutive violations per event)
- HELIX algorithmic benchmarks on Jetson: 64K samp/s anomaly, 156K msg/s log parser
- Cross-device DDS RTT: 1.63 ms mean (PC ↔ Jetson, 50 trials) — source: `hardware_eval_20260403/results/cross_device_latency.txt` (text log, no JSON artifact in `results/`)
- Jetson resource state: 82% idle CPU, 13GB free RAM with GO2 stack

### Inferred from Hardware
- HELIX full-stack overhead on Jetson (projected from PC measurement + algorithmic scaling)
- Long-term rate stability (projected from multiple short captures)
- Adapter viability for sustained monitoring (demonstrated for 60 seconds)
- Thermal headroom for HELIX on Jetson (43–48°C measured vs 85°C throttle point)

### Not Yet Validated
- ~~HELIX running as persistent ROS 2 service on GO2/Jetson~~ → **VALIDATED 2026-04-15: 1780 s on-Jetson run, all success_criteria true. See `GO2_HARDWARE_EVIDENCE.md` § 8 and `HARDWARE_EXPERIMENT_INDEX.md` EXP-15.**
- Fault detection accuracy against known GO2 fault scenarios — **PARTIAL via Session 5 ground-truth injection (EXP-17): log-pattern detection works (~1.8 s), but physical/sensor injections produce no faults absent an adapter layer.**
- Log parser with GO2-specific rules against real GO2 error logs
- Adapter output correctness (are derived metrics semantically meaningful?)
- Sustained operation under thermal stress
- Multi-robot or multi-platform attachability comparison

### Session 5 (2026-04-15) — additional defensible claims
- "HELIX runs as a persistent ROS 2 service on Jetson Orin NX vs live GO2 for ≥30 minutes with no node deaths, no OOM, no thermal throttle." — Strong (EXP-15).
- "End-to-end log-pattern fault detection via `/rosout` works on Jetson with ~1.8 s latency, severity correctly mapped from `log_rules.yaml`." — Strong (EXP-17 inject 1).
- "Without an adapter layer, physical sensor anomalies (LiDAR occlusion, USB device disconnect) and unrelated topic flooding produce 0 HELIX faults — the attachability gap is real on hardware." — Strong (EXP-17 injects 2/3/4).

### Session 5 — claim superseded
- The Session 2 claim that `twist_mux` publishes `/diagnostics` (raising native
  input coverage from 1/4 to 2/4) is **NOT reproduced 2026-04-15** under
  `motion_switcher = normal` across 6 sport-API states. Either the claim was
  mode-dependent (`ai` / `advanced` not yet tested) or the GO2 firmware /
  configuration changed. Until re-verified, the conservative reading is 1/4
  native input coverage (just `/rosout`). See EXP-16.

### Session 6 (2026-04-16) — additional defensible claims
- "All three HELIX fault-type paths are validated end-to-end on hardware.
  LOG_PATTERN was demonstrated in Session 5 (~1.8 s /rosout → /helix/faults).
  CRASH is demonstrated via an external `/helix/heartbeat` publisher that goes
  silent — detector emits a single CRASH FaultEvent at 1.60 s after the third
  missed heartbeat, matching `check_interval_sec=0.5 × miss_threshold=3`.
  ANOMALY is demonstrated on live GO2 data: 60 real FaultEvents in a 20-min
  run across 6 distinct metric labels (utlidar rate metrics, pose drift,
  config-state transitions), on top of a synthetic Z-spike positive control
  (zscore 5.38)." — Strong (PHASE2_SUMMARY §3, §4E).
- "The packaged `helix_adapter` lifecycle stack survives concurrent
  come-here and phoenix load on the same Jetson past the 866 s point at which
  Session 4 crashed, for the full 20-min window with 0 node deaths, 0
  lifecycle transitions, and thermal stable at ~57 °C." — Strong
  (PHASE2_SUMMARY §4A, §4B).
- "`sim_mode:=true` on `helix_adapter.launch.py` remaps the topic_rate_monitor
  subscription from `/utlidar/cloud` to `/utlidar/cloud_throttled`, verified
  end-to-end via topic-info subscriber counts and metric-label emission." —
  Strong (PHASE2_SUMMARY §5).

### Session 7 (2026-04-17) — additional defensible claims
- "The 6-node canonical adapter path exhibits RSS **plateau**, not linear
  leak, over a 1-hour run on the Jetson. Total RSS grew 257 → 282 MB with
  first-half growth 0.717 MB/min and second-half 0.103 MB/min (7× deceleration);
  RSS was flat at 281.77 MB from t ≈ 38 min onward (samples 226, 270, 359 all
  identical). The Phase 2 concern that +0.7 MB/min over 20 min might
  extrapolate badly past 1 hr is retired for the current config." — Strong
  (PHASE3_SUMMARY §2B).
- "Session 1 (2026-04-06) Jetson benchmark numbers reproduce within ±2 % on
  2026-04-17 across anomaly throughput, anomaly latency (mean and p95),
  heartbeat miss latency, log-parser throughput, and log-parser accuracy." —
  Strong (PHASE3_SUMMARY §3).
- "Adapter CPU on the Jetson is configurable from ~48 % of one core to under
  6 % via a single edit to `helix_topic_rate_monitor.topics` — removing
  `/utlidar/imu` (250 Hz) drops that node's CPU 41.3 % → 2.58 % (−94 %) and
  6-node sum 47.6 % → 5.86 % (−88 %), with no effect on the fault-detection
  paths for the other 5 topics." — Strong (PHASE3_SUMMARY §4).

### Session 6 — claim retired
- The Phase 2 hypothesis that the 20-min RSS trend (`+0.7 MB/min`) was a
  `topic_rate_monitor` callback leak on high-rate topics is **refuted** by
  Session 7. Over 1 hr, `topic_rate_monitor` had the *smallest* RSS delta
  (+2.48 MB) despite dominating CPU; growth was spread evenly across all 6
  processes (+2.5 – 5.1 MB each), consistent with ordinary Python heap
  warm-up. See PHASE3_SUMMARY §2C.
