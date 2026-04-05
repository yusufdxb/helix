# HELIX — Fault Detection Results

This document tracks empirical performance measurements for the HELIX fault-sensing stack.

## How to Reproduce

```bash
# Step 1 — start the HELIX sensing stack
source ~/helix_ws/helix_env.sh
ros2 launch helix_bringup helix_sensing.launch.py

# Step 2 — in a second terminal, run the benchmark
source ~/helix_ws/helix_env.sh
python3 benchmark/fault_detection_benchmark.py \
    --trials 20 \
    --baseline-sec 60 \
    --output benchmark/results/run_$(date +%Y%m%d_%H%M)
```

Results are written to `benchmark/results/<run>.csv` (per-trial) and `<run>.json` (summary).

---

## Fault Detection Latency

> **Status: pending first benchmark run.**
> Run `python3 benchmark/fault_detection_benchmark.py` and paste results here.

| Fault Type   | Detection Rate | Mean TTD (ms) | Std TTD (ms) | Min (ms) | Max (ms) | Trials |
|-------------|---------------|--------------|-------------|---------|---------|--------|
| CRASH        | —             | —            | —           | —       | —       | —      |
| ANOMALY      | —             | —            | —           | —       | —       | —      |
| LOG_PATTERN  | —             | —            | —           | —       | —       | —      |

_TTD = time-to-detection: wall time from fault injection trigger to `/helix/faults` publish._

### False Positive Rate

| Observation Window | False Positives | Rate |
|-------------------|----------------|------|
| 60 s baseline      | —              | —    |

---

## Configuration Used

All results above were obtained with:

```yaml
# helix_bringup/config/helix_sensing.yaml
zscore_threshold: 3.0
consecutive_trigger: 3
window_size: 60
heartbeat_timeout_sec: 0.3
miss_threshold: 3
check_interval_sec: 0.5
```

To reproduce a specific result, use the corresponding `benchmark/results/<run>.json`
which records the exact config and timestamp.

---

## Design: How Latency Is Bounded

**Heartbeat / CRASH faults:**
Detection latency is bounded by `check_interval_sec` (default 0.5 s) × `miss_threshold` (default 3).
Theoretical worst-case detection time: `0.5 × 3 = 1.5 s`.

**Anomaly faults:**
Detection fires after `consecutive_trigger` (default 3) consecutive Z-score violations.
Latency depends on metric publish rate on `/helix/metrics`.
At 10 Hz metric publishing: `3 × 100 ms = ~300 ms` minimum TTD.

**Log pattern faults:**
Detection fires on the first matching log line. Latency bounded by `/rosout` publish latency,
typically < 100 ms.

---

## Planned Evaluation (Phase 2)

Once the recovery stack (Phase 2) is integrated, this document will also track:

- Recovery success rate per fault type
- Mean time-to-recovery (MTTR)
- False recovery rate (recovery triggered for a non-fault)
- Verification accuracy (verified success vs. actual recovery)
