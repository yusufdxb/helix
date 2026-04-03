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
Measured on PC (Intel i7-7700 @ 3.60 GHz), 2026-04-03.

| Metric | Value |
|--------|-------|
| Samples to detection (mean) | 3 |
| Detection latency — mean | 0.049 ms |
| Detection latency — p95 | 0.068 ms |
| Trials | 200 |
| Detected | 200/200 |

### AnomalyDetector — Throughput

Single-threaded pure-Python `_process_sample` loop. Window size: 60. Samples: 100,000.
Measured on PC (Intel i7-7700 @ 3.60 GHz), 2026-04-03.

| Metric | Value | Source |
|--------|-------|--------|
| Throughput (PC) | ~81,000 samples/sec | `hardware_eval_20260403/results/jetson_vs_pc_benchmarks.json` |
| Throughput (Jetson Orin NX) | ~64,000 samples/sec | ibid. |
| Wall time (PC) | 1.24 s | `results/standalone_benchmark.md` |

At 100 samples/sec operational load (10 Hz x 10 metrics), the Jetson runs at <0.2% of single-core capacity (636x headroom). Actual ROS 2 node performance will differ due to message serialization, callback scheduling, and runtime overhead.

**Note:** Earlier runs on different hardware reported ~331K samples/sec. The numbers above are from the hardware evaluation session (2026-04-03). The Jetson figures are stored in `hardware_eval_20260403/`, not in the main `results/` directory.

### AnomalyDetector — TPR / FPR (Trivial Baseline)

200 positive trials (spike=100 sigma) + 200 negative trials (noise delta <= 0.15).

| Z-score Threshold | TPR | FPR |
|-------------------|-----|-----|
| 1.5 — 5.0 | 100.0% | 0.0% |

**Caveat:** The perfect separation reflects the extreme gap between normal noise (sigma ~0.08) and injected spikes (~1125 sigma). Any threshold-based detector would achieve these results on this data. See Section 2 for non-trivial evaluation.

### HeartbeatMonitor — Miss Detection Latency

Config: timeout=0.1s, miss_threshold=3, check_interval=0.05s. 30 trials.
Measured on PC (Intel i7-7700), 2026-04-03.

| Metric | Value |
|--------|-------|
| Mean detection latency | 200.7 ms |
| p95 detection latency | 202.0 ms |

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
| 2.0 | 52.0% | 100% | 100% | 100% | 0.5% |
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

| Metric | Value | Source |
|--------|-------|--------|
| Throughput (PC) | ~777,000 msg/sec | `results/log_parser_results.json` (2026-04-02) |
| Elapsed (PC) | 0.013 s | ibid. |
| Throughput (Jetson Orin NX) | ~156,000 msg/sec | `hardware_eval_20260403/results/jetson_vs_pc_benchmarks.json` |

**Note:** The PC throughput above is from `results/log_parser_results.json` (777,029 msg/sec, 10K messages in 0.01287 s). An earlier run on different hardware reported ~248K msg/sec; the difference is due to hardware/run conditions. The Jetson figure comes from a separate benchmark run stored in `hardware_eval_20260403/`.

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

Analysis of the Unitree GO2's ROS 2 topic landscape against HELIX's input requirements, based on live captures from the GO2 (192.168.123.161).

Topic counts vary across captures due to GO2 operating mode:
- 2026-04-02 (Jetson observer): 121 topics
- 2026-04-03 (PC observer, session 1): 103 topics
- 2026-04-03 (PC observer, session 2): 153 topics

The variation reflects which GO2 subsystems are active at capture time (e.g., SLAM, navigation, video streaming).

```bash
python3 scripts/go2_topic_gap_analysis.py         # static analysis
python3 scripts/attachability_matrix.py            # live graph analysis
```

| HELIX Input | GO2 Status (2026-04-03) |
|-------------|------------------------|
| /diagnostics | Available (twist_mux, ~2 Hz) |
| /helix/heartbeat | Not published |
| /helix/metrics | Not published (bridged via passive_adapter.py) |
| /rosout | Available |

**Result:** 2 of 4 HELIX input channels have native data sources on the GO2. A passive adapter (`scripts/passive_adapter.py`) bridges 5 additional topic rate streams and 2 JSON state streams into `/helix/metrics`, enabling HELIX's anomaly detector to operate on live GO2 data. See `docs/GO2_HARDWARE_EVIDENCE.md` for full analysis.

---

## 6. Hardware Evaluation — GO2 + Jetson Orin NX

**Date:** 2026-04-03. **Platform:** Live Unitree GO2 at 192.168.123.161, Jetson Orin NX at 192.168.123.18, PC at 192.168.123.10.

### Adapter-Based Detection on Live GO2

HELIX's anomaly detector ran on the PC while a passive adapter (`scripts/passive_adapter.py`) bridged live GO2 topic rates into `/helix/metrics`. During a 60-second evaluation:

| Metric | Value | Source |
|--------|-------|--------|
| FaultEvents emitted | 4 | `results/helix_overhead_with_adapter.json` |
| Fault source | `/utlidar/cloud` rate fluctuation | ibid. |
| Peak Z-score | 5.52 | ibid., first emitted event |
| Consecutive violations per event | 3 | ibid., each event detail states "3 consecutive samples" |
| RSS memory | 41.7 MB mean | ibid. |
| CPU usage | 22.3% mean (multi-threaded executor) | ibid. |

The LiDAR point cloud topic experienced a rate fluctuation that the adapter converted into a metric stream, and HELIX's Z-score detector flagged as anomalous. Whether this fluctuation represents a genuine sensor fault or normal DDS transport jitter is unknown — no ground-truth labeling exists.

**What this demonstrates:** HELIX's detection logic can flag rate fluctuations in GO2 sensor data when bridged through a lightweight adapter.

**What this does not prove:** That the flagged event was a real fault, or that HELIX would function as a persistent, reliable monitor in deployment. This was a single 60-second controlled evaluation.

### HELIX Node Resource Overhead (Measured)

| Configuration | RSS (mean) | RSS (max) | CPU (mean) | CPU (max) | Faults |
|---------------|-----------|-----------|-----------|-----------|--------|
| 3 HELIX nodes only | 41.4 MB | 41.7 MB | 10.7% | 20.0% | 0 |
| 3 HELIX nodes + adapter | 41.7 MB | 42.1 MB | 22.3% | 49.9% | 4 |

Measured on PC (i7-7700). Overhead on the Jetson Orin NX would differ — see algorithmic benchmark comparison for scaling factors (Jetson runs at 64–79% of PC throughput).

### Cross-Platform Benchmark Comparison

Source: `hardware_eval_20260403/results/jetson_vs_pc_benchmarks.json`

| Metric | PC (i7-7700) | Jetson Orin NX | Ratio |
|--------|-------------|----------------|-------|
| Anomaly throughput | 81K samp/s | 64K samp/s | 0.79x |
| Log parser throughput | 248K msg/s | 156K msg/s | 0.63x |
| Detection latency | 0.049 ms | 0.049 ms | 1.0x |
| Heartbeat miss latency | 200.7 ms | 200.7 ms | 1.0x |

**Note:** The PC log parser throughput here (248K) differs from the value in `results/log_parser_results.json` (777K) because these are separate benchmark runs on different dates. The Jetson figures have no corresponding artifact in `results/` — they are stored only in `hardware_eval_20260403/`.

### GO2 Topic Rate Stability

Measured from bag captures using `scripts/bag_rate_analysis.py`:

| Topic | Rate (Hz) | Jitter (ms) | Stability | Duration | Source |
|-------|-----------|-------------|-----------|----------|--------|
| /utlidar/robot_pose | 18.75 | 1.20 | 99.8% | 34.67s | `results/bag_rate_analysis_single.json` |
| /audiohub/player/state | 3.98 | 0.67 | 100.0% | 34.38s | ibid. |
| /gnss | 1.0 | 1.23 | 100.0% | 34.02s | ibid. |
| /multiplestate | 1.0 | 0.92 | 100.0% | 34.12s | ibid. |

Cross-bag coefficient of variation (CV) for shared topics: <0.01 (highly stable). Source: `results/bag_rate_analysis_cross_3bags.json`.

### Attachability Matrix

Computed from 153 live GO2 topics by `scripts/attachability_matrix.py`:

| Metric | Value |
|--------|-------|
| Native HELIX input coverage | 2/4 (50%) |
| Standard-type topics | 83/153 (54%) |
| Demonstrated adapter topics | 24 |
| Candidate adapter topics (String heuristic) | 30 |
| Topics behind custom msg barrier | 70 |
| with_adapters score | 1.0 (all 4 HELIX inputs fillable) |

Of the 54 adaptable topics, 24 have demonstrated adapter paths (sensor types with known converters) and 30 are heuristic candidates (`std_msgs/msg/String` topics that may contain parseable JSON). The candidate count should be treated as an upper bound, not a confirmed capability.

Full results: `results/attachability_matrix.json`

---

## Reproducing All Results

| Benchmark | Command | ROS 2 Required |
|-----------|---------|---------------|
| Standalone algorithmic | `python3 benchmark_helix.py` | No |
| Realistic anomalies | `python3 scripts/bench_realistic_anomalies.py` | No |
| End-to-end ROS 2 latency | `python3 scripts/bench_e2e_latency.py` | Yes (Humble + helix_msgs) |
| Log parser evaluation | `python3 scripts/bench_log_parser.py` | No |
| GO2 topic gap analysis | `python3 scripts/go2_topic_gap_analysis.py` | No |
| Attachability matrix | `python3 scripts/attachability_matrix.py` | Yes (live graph) |
| Bag rate analysis | `python3 scripts/bag_rate_analysis.py <bag>` | Yes (rosbag2) |
| HELIX overhead | `python3 scripts/measure_helix_overhead.py` | Yes (Humble + helix_msgs) |
| Passive adapter | `python3 scripts/passive_adapter.py` | Yes (live GO2) |
| Unit tests | `colcon test --packages-select helix_core` | Yes (Humble) |

All JSON result artifacts are stored in `results/`.
