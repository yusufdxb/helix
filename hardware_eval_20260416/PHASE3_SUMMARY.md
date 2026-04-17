# HELIX Phase 3 Summary — 2026-04-17

Follow-on to `PHASE2_SUMMARY.md`. Three items from `NEXT_STEPS.md` run in sequence while hardware was up.

Repo: `T7:helix` @ `hardware-eval/2026-04-16`. Jetson workspace unchanged. Session dir: `T7:LABWORK/HELIX/hardware_eval_20260416_adapter_live/jetson_live_copy_phase3/`.

---

## 1. Items run

| # | Item | Outcome |
|---|---|---|
| 1 | 1-hour stability run — RSS-leak question | ✅ **plateau confirmed**, not a leak |
| 2 | Jetson benchmark reproduction (Session 1 numbers) | ✅ within ±2 % across all metrics |
| 3 | Overhead with `/utlidar/imu` excluded | ✅ dominant — topic_rate_monitor drops 41.3 % → 2.58 % CPU |

## 2. 1-hour stability — plateau, not leak

**Run window:** 2026-04-17 15:27:30 → 16:27:30 UTC. 3603.0 s, 360 samples at 10 s, psutil-based sampler + live `/helix/faults` echo. All 6 lifecycle nodes `active [3]` throughout.

### 2A. Lifecycle

**360 / 360 samples at `alive_count == 6`.** Zero crashes, zero transitions, `load1` ≤ 2.55 peak, mean 0.68. Came up from a cold Jetson (just-booted, no come-here, no phoenix).

### 2B. RSS trajectory — the key finding

| Metric | t=0 | t=30 min | t=60 min | Peak |
|---|---:|---:|---:|---:|
| Total RSS (6 nodes) | 257.16 MB | 278.68 MB | 281.77 MB | 281.77 MB |
| Growth rate (half-period) | — | **0.717 MB/min** (0–30 min) | **0.103 MB/min** (30–60 min) | — |
| Mean / peak total CPU% (sum across nodes) | — | — | — | **47.61 / 55.70** |
| Mean topic_rate_monitor CPU% | — | — | — | 41.27 |
| Thermal max | 48.0 °C | 51.0 °C | 52.7 °C | **52.9 °C** |
| `mem_avail` | 14.31 GB | 14.45 GB | 14.33 GB | 14.64 GB peak |

**Deceleration factor 7 ×** between first and second half. Total RSS was already **flat at 281.77 MB by sample 226 (t ≈ 37.7 min)** and identical at samples 270 (t=2702 s) and 359 (t=3593 s).

**Verdict:** the +0.7 MB/min growth that Phase 2 saw over 20 min was the *rising portion* of a settling curve, not a linear leak. At 1 hr the Python-side caches (rolling windows, interning, GC gen promotion) have stabilized. **The Phase 2 "could extrapolate badly at 4+ hr" worry is retired for the current config.**

### 2C. Per-node RSS deltas (0 → 60 min)

| Node | rss_t0 (MB) | rss_end (MB) | Δ (MB) | cpu_mean | cpu_peak |
|---|---:|---:|---:|---:|---:|
| helix_pose_drift_monitor | 51.08 | 56.15 | **+5.07** | 2.32 | 2.90 |
| helix_anomaly_detector | 37.52 | 42.51 | **+4.99** | 2.21 | 4.00 |
| helix_heartbeat_monitor | 35.44 | 40.05 | +4.61 | 0.53 | 0.70 |
| helix_json_state_parser | 37.45 | 41.93 | +4.48 | 0.96 | 2.00 |
| helix_log_parser | 37.38 | 40.36 | +2.98 | 0.32 | 2.00 |
| **helix_topic_rate_monitor** | **58.29** | **60.77** | **+2.48** | **41.27** | **45.70** |

**Surprising:** the Phase 2 hypothesis blamed topic_rate_monitor's high-rate callbacks for the RSS trend ("likely Python GC / cache growth on topic_rate_monitor's high-rate callbacks"). The 1-hour data *refutes* that attribution — topic_rate_monitor had the **smallest** RSS delta (+2.48 MB), despite burning ~40 × more CPU than any other node. Growth is spread approximately evenly across all 6 processes (+2.5 – 5.1 MB each), consistent with ordinary per-process Python heap warm-up rather than a specific callback leak.

### 2D. Faults captured during the run

3020 raw message lines → **151 `fault_type: ANOMALY` events** across 7 distinct metric labels (60 min, idle GO2 on the stand):

| node_name (metric label) | # | Comment |
|---|---:|---|
| `rate_hz/multiplestate` | 49 | 1 Hz topic; small N, high jitter relative to mean |
| `rate_hz/gnss` | 48 | 1 Hz topic, same |
| `rate_hz/utlidar_robot_pose` | 19 | ~19 Hz, natural jitter |
| `rate_hz/utlidar_robot_odom` | 15 | ~150 Hz |
| `rate_hz/utlidar_cloud` | 12 | ~15 Hz |
| `rate_hz/utlidar_imu` | 5 | 250 Hz, very tight |
| `pose/displacement_rate_m_s` | 3 | transient motion events |

- No `CRASH`, no `LOG_PATTERN` — expected, nothing was publishing on `/helix/heartbeat` and no log rules triggered.
- Mode-dependent fault mix: Phase 2 (different GO2 mode, come-here concurrent) was utlidar-heavy; today's 1-hour run was dominated by low-rate topics (`/gnss`, `/multiplestate`). Both are natural statistical noise crossings on Z ≥ 3, not real faults.

## 3. Jetson benchmark reproduction (Session 1 vs today)

Pure Python, no ROS runtime, no GO2 load. Ran immediately after tearing down the stack post-stability run.

| Benchmark | 2026-04-06 | 2026-04-17 | Δ |
|---|---:|---:|---|
| Anomaly detection throughput | 63,433 samp/s | **62,225 samp/s** | −1.9 % |
| Anomaly detection latency (mean) | 0.0492 ms | 0.0488 ms | −0.8 % |
| Anomaly detection latency (p95) | 0.0504 ms | 0.0506 ms | +0.4 % |
| Heartbeat miss latency (mean) | 200.5 ms | 200.3 ms | −0.1 % |
| Heartbeat miss latency (p95) | 200.9 ms | 200.3 ms | −0.3 % |
| Log-parser throughput | 154,798 msg/s | **154,915 msg/s** | +0.08 % |
| Log-parser accuracy (22 cases) | 22/22 | 22/22 | identical |
| Realistic anomalies (seeded 42) | — | byte-for-byte identical | — |

**All within ±2 %.** Session 1 benchmark numbers reproduce cleanly on 2026-04-17.

## 4. Overhead with `/utlidar/imu` subscription removed

**Setup:** after benchmarks, relaunched helix_sensing + helix_adapter from a cold state, set `/helix_topic_rate_monitor/topics` to the 5-topic list (drop `/utlidar/imu`) before configuring, activated all 6. 60-second 1 Hz psutil sampler per node.

| Node | CPU % (mean) with IMU (Phase 3 1-hr run) | CPU % (mean) no-IMU (Phase 3 Task 2) | Δ |
|---|---:|---:|---|
| helix_topic_rate_monitor | **41.27** | **2.58** | **−38.69 pp** (−94 %) |
| helix_anomaly_detector | 2.21 | 1.80 | −0.41 pp |
| helix_pose_drift_monitor | 2.32 | 0.35 | −1.97 pp |
| helix_json_state_parser | 0.96 | 0.73 | −0.23 pp |
| helix_heartbeat_monitor | 0.53 | 0.30 | −0.23 pp |
| helix_log_parser | 0.32 | 0.10 | −0.22 pp |
| **Sum** | **47.61** | **5.86** | **−41.75 pp** (−88 %) |

RSS was essentially unchanged (270.78 MB sum vs 281.77 MB sum — the −11 MB delta is within run-to-run warm-up variance, since the no-IMU run only collected 60 s of samples).

**The 250 Hz `/utlidar/imu` callback alone is responsible for ~94 % of topic_rate_monitor's CPU and ~88 % of the whole adapter's CPU footprint** on this Jetson. The expected 8–15 % landing zone from the plan was too conservative — actual landing is 2–3 %.

**Decision:** README should add a tuning note:

> For deployments where 250 Hz `/utlidar/imu` rate monitoring is not required, drop it from `helix_topic_rate_monitor.topics`. This reduces adapter CPU from ~48 % of one Jetson core to under 6 %, without changing the fault-detection paths for the other 5 topics.

## 5. Phase 3 net score (updated against Phase 2)

| Claim | Phase 2 status | Phase 3 status |
|---|---|---|
| All 6 lifecycle nodes stable | ✅ (20 min, 120/120) | ✅ (1 hr, 360/360) |
| Stability under concurrent apps | ✅ (come-here + phoenix concurrent) | ✅ (cold Jetson, 0 concurrent) — both conditions covered |
| **RSS trajectory is a real leak?** | ⚠️ +0.7 MB/min — couldn't tell from 20 min | ❌ **refuted** — plateaus at ~282 MB by ~40 min |
| **Leak attributed to topic_rate_monitor?** | ⚠️ hypothesized | ❌ **refuted** — topic_rate_monitor had *smallest* RSS growth |
| Session 1 benchmark numbers reproduce | ❌ never re-run | ✅ ±2 % across all metrics |
| Adapter CPU is load-bearing at ~50 %? | ✅ yes, but cause unclear within topic_rate_monitor | ✅ yes, cause pinned: **250 Hz /utlidar/imu callback** |
| Adapter CPU can be reduced by config | ❌ unknown | ✅ **48 % → 6 %** by dropping IMU from topics list |
| 1-hour persistent deployment | ❌ | ✅ (bounded, no crashes, plateaus) |
| Hours-to-days persistent deployment | ❌ | ❌ still out of scope — 1 hr is bounded |

## 6. What still remains unproven

- **Multi-hour / days-scale persistence.** Plateau at 1 hr is not a proof at 24 hr. Growth could resume slowly (page-cache, fragmentation, thermal-driven effects). Next hardware window should run 4 hr overnight.
- **Anomaly-detector behavior during actual GO2 faults.** The 151 anomalies in the 1-hr run are all statistical-noise crossings on a stationary robot — not a genuine lidar dropout or sport-mode transition. That case still requires either (a) an intentional fault injection or (b) capturing a real-world failure event, and stays on the skip list for the current workshop-paper scope.
- **`/diagnostics` mode dependence** unchanged since Phase 2 (GO2 mode-gated, not a HELIX property).

## 7. Artifact tree (Phase 3 additions only)

    T7:LABWORK/HELIX/hardware_eval_20260416_adapter_live/jetson_live_copy_phase3/
    ├── env.sh                                       — unchanged (Phase 2 env)
    ├── logs/
    │   ├── sensing_launch_20260417T152537Z.log       — phase 3 initial launch
    │   ├── adapter_launch_20260417T152537Z.log
    │   ├── lifecycle_phase3_active_20260417T152654Z.txt  — all 6 active [3]
    │   ├── stability_1hr_stdout_20260417T152654Z.log     — "360 samples over 3603.0s"
    │   ├── sensing_launch_task2_20260417T163454Z.log     — task-2 cold relaunch
    │   └── adapter_launch_task2_20260417T163454Z.log
    └── results/
        ├── stability_sampler_1hr.py                  — DURATION=3600 derivative of phase-2 sampler
        ├── stability_1hr.json                        — 360 samples, full record
        ├── stability_1hr_live.json                   — last-write snapshot during run
        ├── faults_stability_1hr_20260417T152654Z.txt — 3020 lines, 151 ANOMALY events
        ├── phase3_active_run_20260417T152654Z.txt    — run metadata (pgid, start, expected end)
        ├── jetson_bench_anomaly_20260417.txt         — Session 1 anomaly benchmark re-run
        ├── jetson_bench_log_parser_20260417T163136Z.txt
        ├── jetson_bench_realistic_anomalies_20260417T163136Z.txt
        └── overhead_noimu_20260417T163454Z.json      — 60 s, 6 nodes, IMU excluded

## 8. Doc-patch backlog — now unblocked

The narrow doc commit queued in Phase 2 §11 can now land. The 1-hour stability result was the gate; it passed. Suggested single commit:

- `README.md`: delete `passive_adapter.py` paragraph; add packaged `helix_adapter` taxonomy; replace the old `<0.5% CPU / <50 MB` projection with the measured **`~48 % of one Jetson core / ~282 MB RSS (plateau at 40 min) for the full 6-node canonical path on GO2 live data`**, plus the new Phase 3 tuning note from §4 above.
- `docs/GO2_HARDWARE_EVIDENCE.md`: add Session 6 (Phase 2) and Session 7 (Phase 3) entries.
- `docs/GO2_ATTACHABILITY_UPDATE.md`: unchanged re-validation needed — today's run didn't re-probe attachability.
- `docs/PAPER_CLAIMS.md`: can now additionally claim (a) 1-hour RSS stability (no leak), (b) Session 1 benchmark numbers reproduce across 11 days, (c) Jetson adapter CPU is configurable 48 % → 6 % via one topics-list edit.

## 9. Reproduction (short)

On Jetson, from `~/helix_live_20260416_p2/`:

1. `source env.sh`
2. `ros2 launch helix_bringup helix_sensing.launch.py &`
3. `ros2 launch helix_bringup helix_adapter.launch.py &`
4. For each of the 6 helix nodes: `ros2 lifecycle set /<node> configure && ros2 lifecycle set /<node> activate`
5. 1-hr run: `setsid nohup python3 -u results/stability_sampler_1hr.py > logs/stability_1hr_stdout.log 2>&1 < /dev/null &`
6. Parallel: `setsid nohup timeout 3605 ros2 topic echo /helix/faults > results/faults_stability_1hr.txt 2>&1 < /dev/null &`

For the no-IMU variant, before `configure` in step 4: `ros2 param set /helix_topic_rate_monitor topics "['/utlidar/robot_odom','/utlidar/robot_pose','/utlidar/cloud','/gnss','/multiplestate']"`.
