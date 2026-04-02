# Results — HELIX Fault Sensing Benchmarks

This document integrates all quantitative evaluation results for the HELIX fault sensing prototype. Results are organized by evaluation type and explicitly scoped.

---

## 1. Standalone Algorithmic Benchmarks

**Script:** `benchmark_helix.py` | **Output:** `results/standalone_benchmark.md`, `benchmark_results.json`

These benchmarks run pure-Python ports of the detection logic — **no ROS 2 runtime, no message transport**. They characterize algorithmic correctness and throughput in isolation.

```bash
python3 benchmark_helix.py   # no ROS 2 required
```

### AnomalyDetector — Detection Latency

Z-score threshold: 3.0, consecutive_trigger: 3, window_size: 60.
Signal: baseline mean=10.0 (noise sigma ~0.08), spike value=100.0.

| Metric | Value |
|--------|-------|
| Samples to detection (mean) | 3 |
| Detection latency — mean | 0.010 ms |
| Detection latency — p95 | 0.014 ms |
| Trials | 200 |
| Detected | 200/200 |

### AnomalyDetector — Throughput

Single-threaded pure-Python `_process_sample` loop. Window size: 60. Samples: 100,000.

| Metric | Value |
|--------|-------|
| Throughput | ~331,000 samples/sec |
| Wall time | 0.30 s |

At 100 samples/sec operational load (10 Hz x 10 metrics), this represents <0.01% of single-core capacity. Actual ROS 2 node performance will differ due to message serialization, callback scheduling, and runtime overhead.

### AnomalyDetector — TPR / FPR (Trivial Baseline)

200 positive trials (spike=100 sigma) + 200 negative trials (noise delta <= 0.15).

| Z-score Threshold | TPR | FPR |
|-------------------|-----|-----|
| 1.5 — 5.0 | 100.0% | 0.0% |

**Caveat:** The perfect separation reflects the extreme gap between normal noise (sigma ~0.08) and injected spikes (~1125 sigma). Any threshold-based detector would achieve these results on this data. See Section 2 for non-trivial evaluation.

### HeartbeatMonitor — Miss Detection Latency

Config: timeout=0.1s, miss_threshold=3, check_interval=0.05s. 30 trials.

| Metric | Value |
|--------|-------|
| Mean detection latency | 200.3 ms |
| p95 detection latency | 200.6 ms |

At production config (miss_threshold=3, check_interval=0.5s), expected latency is ~1.5–2.0s.

---

## 2. Realistic Anomaly Detection Evaluation

**Script:** `scripts/bench_realistic_anomalies.py` | **Output:** `results/realistic_anomaly_results.json`

These benchmarks evaluate the Z-score detector under harder conditions: non-Gaussian noise, gradual drift, transient spikes, and near-threshold anomalies. All use the same pure-Python detection logic (no ROS 2) with seed=42 for reproducibility.

```bash
python3 scripts/bench_realistic_anomalies.py
```

### Scenario 1: Laplace (Heavy-Tailed) Noise

Baseline: 60 samples from Laplace(loc=10.0, scale=1.0). Positive: 3 consecutive spikes at K multiples of scale. 200 trials per condition.

| Z Threshold | K=3 TPR | K=5 TPR | K=8 TPR | K=12 TPR | FPR |
|-------------|---------|---------|---------|----------|-----|
| 2.0 | 52.0% | 100% | 100% | 100% | 0.0% |
| 3.0 | **0.0%** | 50.0% | 100% | 100% | 0.0% |
| 4.0 | 0.0% | 0.5% | 33.0% | 100% | 0.0% |

**Key finding:** The Z-score detector struggles significantly with heavy-tailed noise. At the default threshold (Z=3.0), a 3-sigma spike is undetectable — the Laplace distribution's heavy tails inflate the window's standard deviation, making moderate anomalies invisible. Reliable detection requires spikes at 8+ multiples of the noise scale.

### Scenario 2: Gradual Drift

Baseline: 60 samples from N(10.0, 0.5). Drift: value = 10.0 + R*i + noise for 20 post-baseline samples. Z=3.0, consecutive_trigger=3.

| Drift Rate R | Detection Rate | Mean Samples to Detect |
|--------------|---------------|----------------------|
| 0.1 (0.2 sigma/sample) | 1.5% | 13.3 |
| 0.2 (0.4 sigma/sample) | 53.5% | 13.2 |
| 0.5 (1.0 sigma/sample) | 100% | 6.2 |
| 1.0 (2.0 sigma/sample) | 100% | 4.1 |
| 2.0 (4.0 sigma/sample) | 100% | 3.2 |

**Key finding:** Slow drift largely evades the detector. The rolling window adapts to the drift, raising the running mean and keeping Z-scores low. Detection requires drift of at least ~1 sigma per sample for reliability at Z=3.0. This is expected behavior for a fixed-window Z-score detector.

### Scenario 3: Transient Spike Rejection

Baseline: 60 samples from N(10.0, 0.5). Spikes at 5 sigma above mean. consecutive_trigger=3.

| Spike Pattern | False Alarm Rate |
|---------------|-----------------|
| Single spike | 0.0% (200 trials) |
| Double spike | 0.0% (200 trials) |
| Triple spike (control) | 99.5% TPR |

**Key finding:** The consecutive_trigger mechanism works as designed. Single and double outliers do not trigger false alarms, while three consecutive violations reliably fire.

### Scenario 4: Marginal (Near-Threshold) Anomalies

Baseline: N(10.0, 1.0). Positive: 3 consecutive samples at [3.5, 4.0, 5.0] sigma above mean.

| Z Threshold | TPR | FPR |
|-------------|-----|-----|
| 2.0 | 100% | 0.0% |
| 3.0 | **96.5%** | 0.0% |
| 3.5 | 48.5% | 0.0% |
| 4.0 | 8.0% | 0.0% |

**Key finding:** At Z=3.0, even near-threshold anomalies (smallest spike is 3.5 sigma) are detected at 96.5%. But the threshold is sensitive — Z=3.5 cuts TPR to 48.5%. This shows meaningful trade-offs that the trivial benchmark cannot reveal.

---

## 3. End-to-End ROS 2 Latency

**Script:** `scripts/bench_e2e_latency.py` | **Output:** `results/e2e_latency_results.json`

This benchmark measures latency through the **actual ROS 2 callback path** — not a pure-Python port:

```
publish Float64MultiArray → AnomalyDetector callback → Z-score → FaultEvent → subscriber callback
```

**Includes:** intra-process message passing, rclpy callback scheduling, Z-score computation, FaultEvent publishing.
**Excludes:** inter-process DDS serialization (single-process executor), network transport.

```bash
source /opt/ros/humble/setup.bash && source helix_ws/install/setup.bash
python3 scripts/bench_e2e_latency.py   # requires ROS 2 Humble
```

### Results (50 trials, 50/50 detected)

| Metric | Value |
|--------|-------|
| Mean | 1.16 ms |
| P50 | 1.15 ms |
| P95 | 1.24 ms |
| P99 | 1.25 ms |
| Min | 1.12 ms |
| Max | 1.25 ms |

**Interpretation:** The end-to-end ROS 2 path adds ~1.15 ms over the pure-Python algorithmic latency (~0.01 ms). The overhead is dominated by rclpy callback scheduling and intra-process message delivery. Variance is low (p95/mean ratio = 1.07), indicating consistent performance in a single-process, unloaded environment.

This does not measure multi-process or network latency. In a deployed system with inter-process DDS communication and competing workloads, latency would be higher.

---

## 4. Log Parser Evaluation

**Script:** `scripts/bench_log_parser.py` | **Output:** `results/log_parser_results.json`

Evaluates the log parser's regex-based detection against 5 configured rules. Standalone Python — no ROS 2 required.

```bash
python3 scripts/bench_log_parser.py
```

### Detection Accuracy

22 test cases: 9 true positives, 9 true negatives, 4 edge cases.

| Rule | Precision | Recall |
|------|-----------|--------|
| nav2_costmap_fail | 100% | 100% |
| slam_diverged | 100% | 100% |
| dds_discovery_lost | 100% | 100% |
| transform_timeout | 100% | 100% |
| hardware_estop | 100% | 100% |

Overall: 22/22 correct.

**Edge case examples:** "costmap failed" (no "to initialize") correctly does not match. "estop was disengaged" correctly matches `hardware_estop` — this is a semantic false positive the rule cannot distinguish.

### Throughput

10,000 messages (80% non-matching, 20% matching) against 5 compiled regex patterns.

| Metric | Value |
|--------|-------|
| Throughput | ~754,000 messages/sec |
| Elapsed | 0.013 s |

### Dedup Validation

| Scenario | Expected Faults | Actual | Pass |
|----------|----------------|--------|------|
| 10 messages within 5s window | 1 | 1 | Yes |
| Message after window expires | 1 | 1 | Yes |
| Same rule, different nodes | 2 | 2 | Yes |
| Different rules, same node | 2 | 2 | Yes |

**Limitations:** Detection accuracy is tested against a hand-crafted corpus of 22 messages. The rules are evaluated against their own design intent — not against a corpus of real robot logs. Coverage is limited to the 5 pre-configured rules.

---

## 5. GO2 Platform Gap Analysis

**Script:** `scripts/go2_topic_gap_analysis.py` | **Doc:** `docs/GO2_TOPIC_ANALYSIS.md`

Analysis of the Unitree GO2's ROS 2 topic landscape against HELIX's input requirements, based on a live capture from the GO2 (192.168.123.161) observed via Jetson Orin NX (192.168.123.18) on 2026-04-02.

```bash
python3 scripts/go2_topic_gap_analysis.py
```

| HELIX Input | GO2 Status |
|-------------|-----------|
| /diagnostics | Not published |
| /helix/heartbeat | Not published |
| /helix/metrics | Not published |
| /rosout | Available (5 publishers) |

**Result:** 1 of 4 HELIX input channels has a native data source on the GO2. The gap is bridgeable with adapter nodes but requires meaningful integration work. See `docs/GO2_TOPIC_ANALYSIS.md` for full analysis.

---

## Reproducing All Results

| Benchmark | Command | ROS 2 Required |
|-----------|---------|---------------|
| Standalone algorithmic | `python3 benchmark_helix.py` | No |
| Realistic anomalies | `python3 scripts/bench_realistic_anomalies.py` | No |
| End-to-end ROS 2 latency | `python3 scripts/bench_e2e_latency.py` | Yes (Humble) |
| Log parser evaluation | `python3 scripts/bench_log_parser.py` | No |
| GO2 topic gap analysis | `python3 scripts/go2_topic_gap_analysis.py` | No |
| Unit tests | `colcon test --packages-select helix_core` | Yes (Humble) |

All JSON result artifacts are stored in `results/`.
