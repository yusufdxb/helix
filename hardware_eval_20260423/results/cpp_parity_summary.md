# C++ anomaly detector — 30-min hardware parity (2026-04-23)

**Target doc:** `docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md`
**Baseline doc:** `docs/GO2_HARDWARE_EVIDENCE.md` §15–17 (Session 7)
**Raw JSON:** `results/cpp_parity.json` (891 samples @ 2 s interval)
**Raw bag:** `bags/cpp_parity/cpp_parity_demo/cpp_parity_demo_0.db3` (1 MB)

## Measured

| Metric | C++ (live) | Python baseline (S7) | Change |
|---|---:|---:|---:|
| RSS mean | **19.81 MB** | 45.0 MB | **-56%** |
| RSS max | 20.05 MB | — | — |
| RSS first / last | 18.77 / 20.05 MB | — | +1.28 MB drift, not a leak |
| CPU mean | **0.80%** | ~2.0% | **-60%** |
| CPU max | 0.90% | — | — |

## Targets (design doc)

| Target | Result |
|---|---|
| RSS ≤ 30% of Python (≤ 14 MB) | **FAIL (hit 44% / 19.8 MB)** |
| CPU ≤ 10% of one core | PASS by 12× margin |

Overall: verdict from the automated script says **FAIL** because RSS
ratio target (30% of Python) was missed — actual 44%. But it's still a
substantial reduction (2.27× less RSS than Python) and CPU target
cleared by an order of magnitude.

Design-doc "expected C++ RSS: 8–10 MB" was optimistic; actual 19.8 MB
on this Jetson likely reflects rclcpp runtime overhead + per-metric
rolling window allocations (6 metrics × 60-sample windows) that the
estimate didn't model.

## Fault stream health under C++

Captured in `cpp_parity_demo` bag over the same 30 min:

| Topic | Count | Rate |
|---|---:|---|
| /helix/faults | 33 | 1.1/min natural idle |
| /helix/recovery_hints | 33 | R1 matched 1:1 on utlidar metrics |
| /helix/cmd_vel | 10,877 | 6.04 Hz effective — STOP active ~30% of run |

R1 filter from the earlier fix holds under the C++ emission path —
every utlidar-family ANOMALY produced a STOP_AND_HOLD hint, and
non-utlidar ANOMALYs (gnss, multiplestate) were correctly ignored.

## Follow-up

1. **Don't flip `use_cpp_anomaly` default yet.** The 44% RSS is a real
   win but the design doc gated promotion on the 30% target. Options:
   - Accept the current ratio as pragmatic-good-enough and retune the
     design doc target.
   - Profile the C++ RSS breakdown (rclcpp vs node code vs window
     allocations) and shrink if possible.
2. **Retune `zscore_threshold`.** 33 ANOMALYs in 30 min of idle
   standing is still noisy. Raise to 4.0 or 5.0, or add a minimum
   duration gate before ANOMALY emission.
3. **Session-specific Python control.** The 45 MB Python baseline
   comes from Session 7 notes, run under different conditions. A
   10-min Python re-sample this session would give stricter
   apples-to-apples — but the magnitude of the C++ win (-56% RSS,
   -60% CPU) dwarfs any conceivable baseline noise, so not urgent.
