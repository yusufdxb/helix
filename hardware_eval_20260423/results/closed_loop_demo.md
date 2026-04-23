# HELIX Session 8 — Closed-loop hardware demo (post R1 fix)

**Date:** 2026-04-23
**Bag:** `bags/post_fix_demo/post_fix_demo_0.db3` (75 MB, 7m19s)
**Fix branch:** `fix/r1-anomaly-schema-mismatch` @ `e821b47` (T7 repo_clone)
**Jetson commit:** unpushed rebuild, matches T7 branch content

## Result

**End-to-end closed loop validated on live GO2 hardware.**

| Topic | Messages | Interpretation |
|---|---|---|
| `/helix/faults` | 30 | ANOMALY events from SENSE (`rate_hz/*` metrics) |
| `/helix/recovery_hints` | 14 | 9 × R1 STOP_AND_HOLD, 5 × R2 RESUME |
| `/helix/recovery_actions` | 14 | audit log: 7 × ACCEPTED, 7 × SUPPRESSED_COOLDOWN |
| `/helix/cmd_vel` | 3064 | zero-twist published @20 Hz during STOP states |

Of the 30 faults, **R1 correctly matched only utlidar-family metrics**
(robot_odom × 6, imu × 4). Non-utlidar ANOMALYs (`rate_hz/gnss`,
`rate_hz/multiplestate`) produced no hints — R1's prefix filter works.

R2 RESUME rule fired 5× after the 3.1 s anomaly-clear window. Of those,
2 were ACCEPTED and 3 SUPPRESSED_COOLDOWN because they arrived inside
the recovery node's 5 s cooldown.

## Bug discovered + fixed mid-session

**Root cause:** `helix_diagnosis/rules.py` R1 guard was out of sync with
`helix_core/anomaly_detector.py`'s emission:

| Field | Emitter writes | R1 was reading |
|---|---|---|
| context_keys | `['metric_name', ...]` | looked up `'metric'` |
| value | `'rate_hz/utlidar_robot_odom'` etc. | compared to `'utlidar_rate'` literal |
| correlation id | `node_name` (FaultEvent has no `fault_id`) | `fault.fault_id` (AttributeError if reached) |

Net: R1 returned None on every live ANOMALY. Closed loop stuck at
diagnosis tier. Prior sessions (5/6) hadn't exercised R1 with live
rate metrics (Session 6 explicitly: "No artificial rate perturbation
was applied"), so the bug lay dormant.

**Fix (`e821b47`):**
- `_metric_name` accepts both `metric_name` and `metric` keys.
- R1 matches `rate_hz/utlidar*` prefix OR legacy `utlidar_rate` literal.
- `_fault_source_id(fault)` returns `fault.node_name`, used by R1/R3/R4
  HintShape (FaultEvent has no fault_id).
- Test fixtures updated in `test_rules.py` + `test_diagnosis_node.py`
  to match the live emitter schema.

Tests: **16/16 pass on Jetson** (`pytest` under real `helix_msgs` import).

## Follow-up items (not done this session)

1. **Tune `zscore_threshold`.** With the fix live, R1 now fires on natural
   idle jitter of `/utlidar/robot_odom` and `/utlidar/imu`. In 7m19s of
   idle standing, 9 STOP_AND_HOLD events landed. Raise threshold from
   3.0 → 4.0 or 5.0, or add a minimum-duration gate.
2. **Stale-topic detection.** `topic_rate_monitor` emits NaN when a topic
   goes silent. `anomaly_detector` rejects NaN (z-score of NaN is NaN,
   fails threshold check). A rate-drop injection (e.g., UDP block)
   therefore produces **zero** ANOMALY today. Add either:
   - A dedicated stale-topic fault path in `topic_rate_monitor`, OR
   - Treat NaN as a high-severity rate anomaly in `anomaly_detector`.
3. **Wire `/helix/cmd_vel` to twist_mux fallback.** 0 subscribers today
   — the "robot holds" claim is logically correct but physically vacuous.
   Phase 2 work.
4. **Push fix branch + open PR.** Branch `fix/r1-anomaly-schema-mismatch`
   committed locally on T7. Needs GitHub push + PR review. Kept off
   main until reviewed.

## Operator notes

- `body_height` 0.315 m at start (mode 0 standing, threshold 0.25 m ✓).
- No Damp / StandDown commands issued.
- GO2 did not physically move during the test (`/helix/cmd_vel` has
  0 subscribers; zero-twist published into a void).
- Robot remained in sport mode 0 throughout.

## Artifacts

- `bags/post_fix_demo/` — demo bag (75 MB)
- `notes/SESSION_NOTES.md` — running narrative
- `logs/jetson_colcon_build.log` — build output
- `logs/jetson_prestash_RESULTS_benchmark.patch` — pre-sync stash backup
- `logs/decode_faults.py` + `decode_all.py` — bag inspection helpers
  (kept under `/tmp` on lab PC)

## Repo state

- T7 `repo_clone` branch `fix/r1-anomaly-schema-mismatch` @ `e821b47`
- Jetson `~/yusuf/helix` rebuilt with branch content (not in repo HEAD
  — local install/symlink build picks up the edited src/ files)
- Main unchanged at `4c43778`
