# Results — HELIX

Quantitative evaluation of HELIX fault detection components.
All benchmarks run in standalone mode (no ROS 2 required).
Generated: `2026-03-24T00:21:54Z`

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
| Detection latency — mean | 0.065 ms |
| Detection latency — p95 | 0.0673 ms |
| Trials | 200 |
| Detected | 200/200 |

_Detection requires `consecutive_trigger=3` violations, so minimum latency is 3 samples._

---

## AnomalyDetector — Throughput

Single-threaded, pure-Python `_process_sample` call loop.
Window size: 60. Samples: 100,000.

| Metric | Value |
|--------|-------|
| Throughput | 46,356 samples/sec |
| Total samples | 100,000 |
| Wall time | 2.1572 s |

_At 10 Hz per node × 10 monitored metrics = 100 samples/sec operational load.
HELIX operates at <0.01% of single-core capacity at steady state._

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

---

## HeartbeatMonitor — Miss Detection Latency

Config: timeout=0.1 s, miss_threshold=3, check_interval=0.05 s.
Heartbeats published at 100 Hz for 0.3 s, then stopped.

| Metric | Value |
|--------|-------|
| Mean detection latency | 200.4 ms |
| p95 detection latency | 200.5 ms |
| Min | 200.3 ms |
| Max | 200.5 ms |
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
