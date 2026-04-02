# Results — HELIX Fault Sensing Benchmarks

## Benchmark Scope

All results in this document are from **standalone synthetic benchmarks**. The benchmark script (`benchmark_helix.py`) runs pure-Python ports of the HELIX detection logic — no ROS 2 runtime, no `rclpy`, no message transport is involved.

These measurements characterize the **algorithmic performance of the detection logic in isolation**. They do not reflect end-to-end ROS 2 pipeline latency, message serialization overhead, or performance under real robot workloads. Throughput numbers represent single-core CPython performance.

The TPR/FPR sweep evaluates detection accuracy against synthetic Gaussian data with known injected anomalies, not against real-world fault traces.

Generated: `2026-04-02T22:36:31Z`

---

## How to Reproduce

```bash
# From repo root
python3 benchmark_helix.py
# Outputs: benchmark_results.json, RESULTS.md
```

---

## AnomalyDetector — Detection Latency

Z-score threshold: 3.0 (default), consecutive_trigger: 3, window_size: 60.
Signal: baseline mean=10.0 (noise σ≈0.08), spike value=100.0.

| Metric | Value |
|--------|-------|
| Samples to detection (mean) | 3 |
| Detection latency — mean | 0.0098 ms |
| Detection latency — p95 | 0.0141 ms |
| Trials | 200 |
| Detected | 200/200 |

_Detection requires `consecutive_trigger=3` violations, so minimum latency is 3 samples._

---

## AnomalyDetector — Throughput

Single-threaded, pure-Python `_process_sample` call loop.
Window size: 60. Samples: 100,000.

| Metric | Value |
|--------|-------|
| Throughput | 335,500 samples/sec |
| Total samples | 100,000 |
| Wall time | 0.2981 s |

_At 10 Hz per node × 10 monitored metrics = 100 samples/sec operational load.
HELIX operates at <0.01% of single-core capacity at steady state._

Note: actual ROS 2 node performance will differ due to message serialization, callback scheduling, and other runtime overhead.

---

## AnomalyDetector — TPR / FPR Sweep

200 positive trials (3 consecutive spikes, spike=100σ above baseline) +
200 negative trials (mild noise, Δ≤0.15 from baseline mean).

| Z-score Threshold | TPR | FPR |
|-------------------|-----|-----|
| 1.5 | 100.0% | 0.0% |
| 2.0 | 100.0% | 0.0% |
| 2.5 | 100.0% | 0.0% |
| 3.0 | 100.0% | 0.0% |
| 3.5 | 100.0% | 0.0% |
| 4.0 | 100.0% | 0.0% |
| 5.0 | 100.0% | 0.0% |

_Default Z=3.0: TPR=100.0%, FPR=0.0%._

The perfect separation across all thresholds reflects the large gap between normal noise (sigma ~0.08) and injected spikes (~1125 sigma above baseline). Any threshold-based detector would achieve perfect results on this data. Real-world fault signatures would produce less clean separation, particularly with non-Gaussian noise, gradual drift, or near-threshold anomalies.

---

## HeartbeatMonitor — Miss Detection Latency

Config: timeout=0.1 s, miss_threshold=3, check_interval=0.05 s.
Heartbeats published at 100 Hz for 0.3 s, then stopped.

| Metric | Value |
|--------|-------|
| Mean detection latency | 200.3 ms |
| p95 detection latency | 200.3 ms |
| Min | 200.2 ms |
| Max | 200.3 ms |
| Trials | 30 |

_At production config (miss_threshold=3, check_interval=0.5 s), expected latency ≈ 1.5–2.0 s._

---

## Test Coverage

Run the unit test suite (requires ROS 2 Humble + rclpy):

```bash
cd src/helix_core
python3 -m pytest test/ -v
```

| Test | Status |
|------|--------|
| `test_anomaly_detection` | ✅ |
| `test_crash_detection` | ✅ |

See `TESTING.md` for full test setup instructions.
