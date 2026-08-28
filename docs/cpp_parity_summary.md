# C++ anomaly detector: 30-min hardware parity (2026-04-23)

**Target doc:** `docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md`
**Baseline doc:** `docs/GO2_HARDWARE_EVIDENCE.md` §15-17 (Session 7)
**Baseline numbers:** the per-node table in §15, `helix_anomaly_detector` row.
Earlier revisions of this page scored the port against round narrative
figures ("45.0 MB", "~2.0%") taken from prose rather than from that
table; every Python column below is now the measured value.
**Raw JSON:** `results/cpp_parity.json` (891 samples @ 2 s interval)
**Raw bag:** `cpp_parity_demo_0.db3` (1 MB, archived offline)

## Measured

The Python baseline is a 1-hour cold-start run, so it has two defensible
reference points: RSS at t=0 (37.52 MB) and RSS after the heap plateau at
t=60 min (42.51 MB). The C++ run is 30 min, so it is compared against both.

| Metric | C++ (live, 30 min) | Python S7 (§15) | Change |
|---|---:|---:|---:|
| RSS mean | **19.81 MB** | 42.51 MB (t=60 min, plateau) | **-53.4%** (ratio 46.6%) |
| RSS mean | **19.81 MB** | 37.52 MB (t=0, cold start) | **-47.2%** (ratio 52.8%) |
| RSS max | 20.05 MB | 42.51 MB | -52.8% |
| RSS first / last | 18.77 / 20.05 MB | 37.52 / 42.51 | C++ drift +1.28 MB vs Python +4.99 MB |
| CPU mean | **0.80%** | 2.21% | **-63.8%** (ratio 36.2%) |
| CPU max | 0.90% | 4.00% | -77.5% (ratio 22.5%) |

## Targets (design doc)

| Target | Result |
|---|---|
| RSS <= 30% of Python (<= 12.75 MB against the 42.51 MB plateau) | **FAIL (46.6% / 19.81 MB)** |
| CPU <= 10% of one core | **PASS** (0.80% mean, 12x margin) |

Overall: **FAIL** on the RSS target. The design doc's per-node figure of
"~47 MB Python" was the 6-node mean (281.77 MB / 6), not the anomaly
detector itself; the detector's own baseline is 37.52-42.51 MB, so the
30% gate is 11.26-12.75 MB rather than 14 MB. Scoring against the real
row makes the gate slightly harder, not easier, and the port misses it
either way (46.6% against the plateau, 52.8% against cold start).

The reduction is still real: 2.15x less RSS than the plateau baseline,
and CPU cleared by an order of magnitude.

Design-doc "expected C++ RSS: 8-10 MB" was optimistic; actual 19.81 MB
on this Jetson likely reflects rclcpp runtime overhead plus per-metric
rolling window allocations (6 metrics x 60-sample windows) that the
estimate did not model.

## Fault stream health under C++

Captured in `cpp_parity_demo` bag over the same 30 min:

| Topic | Count | Rate |
|---|---:|---|
| /helix/faults | 33 | 1.1/min natural idle |
| /helix/recovery_hints | 33 | R1 matched 1:1 on utlidar metrics |
| /helix/cmd_vel | 10,877 | 6.04 Hz effective, STOP active ~30% of run |

R1 filter from the earlier fix holds under the C++ emission path, 
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
3. **Session-specific Python control.** The Python baseline is the
   Session 7 (2026-04-17) 1-hour cold-start run, and the C++ run is a
   30-min Session 8 (2026-04-23) run. The two differ in duration, in
   GO2 operating mode, and in what else was resident on the Jetson, so
   this is a cross-session comparison, not a controlled A/B. A
   same-session Python re-sample is the only way to remove that
   confound. The measured gap (-53% RSS, -64% CPU) is large relative to
   the +4.99 MB / 1 hr of drift seen inside the Python run itself, but
   that bounds only warm-up noise, not session-to-session variation.
   **Still owed, and the reason the RSS FAIL verdict should not be
   retuned away on the strength of these numbers alone.**
