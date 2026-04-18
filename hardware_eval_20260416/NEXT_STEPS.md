# HELIX — Next Steps (after Phase 3)

Phase 3 (2026-04-17) closed the three validation items from the previous NEXT_STEPS. See `PHASE3_SUMMARY.md` for the full record. This file is the post-Phase-3 plan.

Repo: `T7:helix` @ `hardware-eval/2026-04-16`, tip `a62cd32`.
Session dir: `T7:LABWORK/HELIX/hardware_eval_20260416_adapter_live/jetson_live_copy_phase3/`.
Trigger phrase for resume: **"run HELIX next steps"**.

---

## What Phase 3 settled

| Item | Before | After Phase 3 |
|---|---|---|
| 1-hr RSS trajectory | +0.7 MB/min over 20 min, leak-vs-plateau unknown | **plateau at 282 MB by t≈38 min** (0.72 → 0.10 MB/min deceleration) |
| Leak blamed on topic_rate_monitor? | hypothesized | **refuted** — tr_m had smallest RSS delta |
| Session 1 benchmark numbers reproduce? | never re-run since 2026-04-06 | **±2 %** across anomaly/log_parser/heartbeat/realistic |
| Can adapter CPU be tuned down? | unknown | **yes: 48 % → 6 %** by dropping `/utlidar/imu` from topics |

## Remaining work (priority order)

### 1. Doc-patch commit — NO hardware needed (P0)

`PHASE2_SUMMARY §11` + `PHASE3_SUMMARY §8` describe this commit. Everything is gated-pass now. One narrow commit on `hardware-eval/2026-04-16`:

- `README.md`
  - delete the `passive_adapter.py` paragraph in "Research Context"
  - add the packaged `helix_adapter` → `/helix/metrics` link with the 14-label taxonomy
  - replace the `<0.5% CPU / <50 MB` projection with the real `~48 % of one Jetson core / ~282 MB RSS (plateau at ~40 min)`
  - add the Phase 3 tuning note (IMU-excluded = 6 % CPU)
- `docs/GO2_HARDWARE_EVIDENCE.md`: add Session 6 (Phase 2, 2026-04-16) and Session 7 (Phase 3, 2026-04-17) entries
- `docs/GO2_ATTACHABILITY_UPDATE.md`: replace "2/4 native" with "1–2/4 native, mode-dependent; /rosout is always-on, /diagnostics requires twist_mux mode"
- `docs/PAPER_CLAIMS.md`: claim now covers (a) all three fault-type paths validated on hardware, (b) 1-hour RSS stability with no leak, (c) Session 1 benchmark numbers reproduce 11 days later, (d) adapter CPU is one-topic-list-edit tunable from 48 % → 6 %

**Time:** ~30 min desk-work. No hardware access required.

### 2. 4-hour overnight stability — OPTIONAL hardware ask (P2)

**Gap:** 1-hr plateau is clean, but plateau at 1 hr is not a guarantee at 24 hr. Page-cache, fragmentation, thermal-driven effects could still resume growth slowly.

**What:** same `stability_sampler_1hr.py` with `DURATION = 14400` (4 hr). Runs detached, no babysitting. Needs GO2 powered up on the network the whole time.

**Expected:** total RSS stays within ±5 MB of the 282 MB Phase 3 endpoint. If it drifts up > 10 MB, there's a slower second-order leak worth profiling.

**Time:** 4 hr wall. Zero operator time during the run.

### 3. Real GO2-fault path validation — NOT planned (skip list)

- Lidar dropout / sport-mode transition / ethernet disconnect — requires inducing unsafe robot states. Still on the skip list from Phase 2 §9.
- Heartbeat input from a real monitored node — no such node exists on the GO2; making one is a code task, not a validation task. Defer to a dedicated PR.

## Reproduction cheat-sheet (unchanged from Phase 3 §9)

Jetson at `unitree@192.168.0.2` (password `123`). From `~/helix_live_20260416_p2/`:

```
source env.sh
ros2 launch helix_bringup helix_sensing.launch.py &
ros2 launch helix_bringup helix_adapter.launch.py &
# wait ~15 s for node discovery, then:
for n in helix_heartbeat_monitor helix_anomaly_detector helix_log_parser \
         helix_topic_rate_monitor helix_json_state_parser helix_pose_drift_monitor; do
    ros2 lifecycle set /$n configure
    ros2 lifecycle set /$n activate
done
```

For the IMU-excluded variant, set `/helix_topic_rate_monitor/topics` before `configure`.

## Finishing artifacts (if hardware is touched)

1. Rsync `~/helix_live_20260416_p2/` → `T7:LABWORK/HELIX/hardware_eval_20260416_adapter_live/jetson_live_copy_phaseN/`
2. Write `PHASE<N>_SUMMARY.md` in `hardware_eval_20260416/`
3. Commit on `hardware-eval/2026-04-16`

## Skip list (unchanged)

- GO2 mode-change anomaly scenarios (unsafe)
- Heartbeat from a real monitored node (code change, not validation)
- Cross-device DDS latency re-measurement (Session 1 0.81 ms, no reason to expect drift)
