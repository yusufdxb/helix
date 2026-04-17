# Evaluation Plan — HELIX

Practical evaluation roadmap for a credible workshop paper submission. Organized by what has been validated, what is missing, and what would strengthen the contribution.

---

## Already Evaluated

These benchmarks exist in `benchmark_helix.py` and results are recorded in `benchmark_results.json` / `RESULTS.md`.

| What | Method | Result | Limitation |
|------|--------|--------|------------|
| Detection latency (AnomalyDetector) | 200 trials, step spike from 10.0 to 100.0 | Mean 0.065 ms, 3 samples to detect | Synthetic signal only; spike is ~1125 sigma above baseline |
| Throughput | 100K samples, single-core Python | ~46K samples/sec | Pure Python port, not the ROS 2 node path |
| TPR/FPR sweep | 200 pos + 200 neg trials across Z thresholds 1.5--5.0 | 100% TPR, 0% FPR at all thresholds | Trivially separable: spike is extreme, noise is near-zero variance |
| Heartbeat miss detection | 30 trials, timeout=0.1s, miss_threshold=3 | ~200 ms mean | Pure Python timer simulation, not ROS 2 timer callbacks |
| Unit tests | rclpy-based tests for anomaly_detector, heartbeat_monitor, log_parser | All passing | Exercises node lifecycle, not full integration |

### Key caveat

The TPR/FPR result (100%/0% across all thresholds) reflects the test design, not real-world performance. The positive signal is a 100.0 step change against a baseline of ~10.0 with standard deviation ~0.08. Any threshold-based detector would achieve perfect separation on this data. The negative trials use perturbations of at most 0.15 from the mean — well within 1 sigma. This benchmark validates that the implementation is correct, not that it performs well under realistic conditions.

---

## Missing for Credible Workshop Paper (Minimal)

These experiments address gaps that a reviewer would likely flag.

### 1. Non-Gaussian noise profiles

**Why:** The current benchmark uses a near-deterministic baseline (10.0 + cyclic 0.1 offset). Real sensor data has heavier tails, multimodal distributions, and non-stationary noise.

**Experiment:**
- Generate baseline signals from uniform, Laplace (heavy-tailed), and bimodal Gaussian mixture distributions
- Inject the same anomaly magnitudes (at 3, 5, 10 sigma relative to the noise distribution)
- Measure TPR/FPR for each noise profile across the same Z-score thresholds
- Report where the Z-score detector breaks down (expected: heavy-tailed noise raises FPR)

**Effort:** ~2 hours. Extend `bench_tpr_fpr` with configurable noise generators.

### 2. Gradual drift anomalies

**Why:** The current benchmark only tests step changes (instant jump to 100.0). Real faults often manifest as gradual drift — e.g., a sensor slowly going out of calibration.

**Experiment:**
- Generate ramp anomalies: baseline + linear drift over N samples
- Measure detection latency as a function of drift rate
- Identify the minimum drift rate detectable at Z=3.0 within a 60-sample window

**Effort:** ~1.5 hours. New benchmark function with parameterized drift rate.

### 3. False positive rate under realistic noise

**Why:** The current FPR test uses noise that is far too small to trigger any threshold. A 0% FPR is not informative.

**Experiment:**
- Generate negative trials where normal noise occasionally produces values at 2--3 sigma from the running mean
- Measure FPR at Z=3.0 with noise drawn from N(0, 1) (not the current near-zero-variance signal)
- Report FPR as a function of window size and consecutive_trigger setting

**Effort:** ~1 hour. Modify negative trial generation in `bench_tpr_fpr`.

### 4. Memory and CPU profiling of ROS 2 nodes

**Why:** The benchmark measures pure-Python throughput, but the actual nodes run inside rclpy with timer callbacks, message serialization, and topic I/O. Resource usage under ROS 2 is unmeasured.

**Experiment:**
- Launch the sensing stack with the fault injector
- Use `ros2 topic hz` and `ros2 topic delay` to measure message rates
- Profile with `py-spy` or `cProfile` for CPU usage per node
- Monitor RSS memory over a 5-minute run with `psutil` or `/proc/[pid]/status`

**Effort:** ~3 hours. Requires a ROS 2 Humble environment.

### 5. Multi-metric interaction

**Why:** In production, multiple sensors may fail simultaneously. The current benchmark tests one metric stream at a time.

**Experiment:**
- Run the anomaly detector with 10+ concurrent metric streams
- Inject anomalies on 1, 3, and 5 streams simultaneously
- Verify that detection on one stream does not affect latency or accuracy on others
- Measure throughput degradation as stream count increases

**Effort:** ~1.5 hours. Extend `bench_throughput` with multi-metric support.

### 6. Log parser benchmark

**Why:** The log parser node is untested for performance. Pattern matching throughput and latency are unknown.

**Experiment:**
- Port the log parser's regex matching logic to a standalone benchmark
- Generate synthetic log streams at 100, 1K, 10K lines/sec
- Measure: pattern match latency, throughput, and false match rate with noisy log data
- Test with the default rule set from the YAML config

**Effort:** ~2 hours. New benchmark function mirroring the anomaly detector structure.

---

## Stretch Experiments (Stronger Paper)

These would strengthen the contribution but are not strictly required for a workshop paper.

### 7. ROS 2 integration benchmark: end-to-end latency

Measure the time from publishing a metric on `/helix/metrics` to receiving the corresponding `FaultEvent` on `/helix/faults`. This captures serialization, scheduling, and callback overhead that the standalone benchmark omits.

### 8. Gazebo simulation with simulated faults

Run a robot model (e.g., TurtleBot3) in Gazebo, inject sensor faults (frozen IMU, drifting odometry), and measure HELIX detection performance against known ground truth.

### 9. Comparison with ROS 2 built-in diagnostics

Benchmark against `diagnostic_aggregator` and `diagnostic_updater` — the standard ROS 2 approach to health monitoring. Compare: detection latency, configurability, and message expressiveness.

### 10. On-device profiling on Jetson Orin

Run the sensing stack on a Jetson Orin NX (16 GB) and profile CPU/memory/power. This validates the "designed for Jetson Orin" claim and provides deployment-relevant numbers.

### 11. Real ROS 2 navigation stack logs

Feed actual `nav2` or `move_base` log output through the log parser to measure detection accuracy on real-world log patterns rather than synthetic ones.

---

## Suggested Priority Order

For a workshop paper deadline, work through these in order:

1. Fix the TPR/FPR methodology (items 1, 2, 3) -- highest reviewer risk
2. Log parser benchmark (item 6) -- covers the third untested node
3. Multi-metric interaction (item 5) -- quick and addresses scalability
4. ROS 2 node profiling (item 4) -- grounds the "ROS 2 middleware" framing
5. Stretch items as time allows

**Estimated total for items 1--6:** ~11 hours of implementation and measurement.
