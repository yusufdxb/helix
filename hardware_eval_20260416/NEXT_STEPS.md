# HELIX — Next Steps (while hardware is up)

Pending validation items from the 2026-04-16 session. Listed in priority order. Pick up when hardware access is available.

Repo: `T7:LABWORK/HELIX/repo_clone` @ `26cca7d`
Last session dir: `T7:LABWORK/HELIX/hardware_eval_20260416_adapter_live/`
Trigger phrase for resume: **"run HELIX next steps"**

---

## 1. 1-hour stability run — RSS-leak question

**Gap:** Phase 2 stability saw RSS grow 250 → 263 MB over 20 min (+0.7 MB/min). Could be a real leak in `topic_rate_monitor` (Python callbacks on high-rate topics) or a plateau. 20 min is too short to tell.

**What:** Same sampler as Phase 2 (`stability_sampler.py`), `DURATION = 3600`, 10 s interval → ~360 samples. Plus `/helix/faults` echo for the hour.

**Expected signal:**
- Plateau → RSS asymptotes around 270–290 MB. No action needed.
- Leak → RSS grows linearly past 300 MB at 1 hr, extrapolates worse at 4+ hr. Action: file a leak issue, profile `topic_rate_monitor` with `tracemalloc`.

**Risk:** Low. Phase 2 already survived 20 min with come-here + phoenix concurrent. Launch with `setsid nohup` — runs detached even if SSH drops or the operator leaves the lab.

**Setup:**
    cd ~/helix_live_20260416_p2     # reuse phase 2 env + helix install
    source ./env.sh
    # relaunch sensing + adapter + activate all 6 (see Phase 2 section A)
    sed -i 's/DURATION = 1200/DURATION = 3600/' results/stability_sampler.py
    setsid nohup python3 results/stability_sampler.py > logs/stability_1hr_stdout.log 2>&1 < /dev/null &
    setsid nohup timeout 3605 ros2 topic echo /helix/faults > results/faults_stability_1hr.txt 2>&1 < /dev/null &

Rsync to `hardware_eval_20260416_adapter_live/jetson_live_copy_phase3/` on return.

**Time:** ~60 min wall, fully detached.

---

## 2. Jetson benchmark reproduction (cross-session)

**Gap:** Session 1 (2026-04-03) published 64K samp/s anomaly throughput and 156K msg/s log-parser throughput on the Jetson. No re-run since. Paper-claim reproducibility question.

**What:** Re-run the existing benchmark scripts on the Jetson with the current build:

    cd ~/helix_ws
    source ~/helix_live_20260416_p2/env.sh
    python3 benchmark_helix.py            > ~/helix_live_20260416_p2/results/jetson_bench_anomaly_20260417.txt
    python3 scripts/bench_log_parser.py   > ~/helix_live_20260416_p2/results/jetson_bench_logparser_20260417.txt
    python3 scripts/bench_realistic_anomalies.py > ~/helix_live_20260416_p2/results/jetson_bench_realistic_20260417.txt

Compare to `results/jetson_benchmark_20260406.txt` in the repo. Expect within ±5 % of the 2026-04-03 / -06 numbers.

**Risk:** None. No ROS runtime, no GO2, no adapter. Pure Python.

**Time:** ~5 min.

---

## 3. Overhead with IMU subscription removed — tuning recommendation

**Gap:** Phase 2 showed `helix_topic_rate_monitor` alone consumes ~42 % of one Jetson core, driven primarily by the 250 Hz `/utlidar/imu` callback (vs 15 Hz cloud, 19 Hz pose, 148 Hz odom, 1 Hz gnss + multistate). Unclear how much IMU specifically contributes vs. the other high-rate sub.

**What:** Launch the adapter with a narrower `topics:` list excluding `/utlidar/imu`, measure new overhead. If total drops to ~10 %, that's a concrete config recommendation for deployers.

**Setup:** Create a one-off params override:

    cat > /tmp/no_imu_params.yaml << 'YAML'
    helix_topic_rate_monitor:
      ros__parameters:
        window_sec: 5.0
        publish_period_sec: 0.5
        topics:
          - "/utlidar/robot_odom"
          - "/utlidar/robot_pose"
          - "/utlidar/cloud"
          - "/gnss"
          - "/multiplestate"
    helix_json_state_parser: { ros__parameters: { publish_period_sec: 0.5, sources: ["/gnss|gnss|satellite_total,satellite_inuse,hdop|", "/multiplestate|go2_state|volume,brightness|obstaclesAvoidSwitch,uwbSwitch"] } }
    helix_pose_drift_monitor: { ros__parameters: { topic: "/utlidar/robot_pose", stale_sec: 5.0, min_dt: 0.001, publish_period_sec: 0.5, metric_name: "pose/displacement_rate_m_s" } }
    YAML
    # then manually run each adapter node with --params-file /tmp/no_imu_params.yaml
    # (or pass through launch with a params override arg)

Run the same 60 s psutil sampler from Phase 1 (`overhead_60s_v2.json` as template).

**Expected:** `topic_rate_monitor` CPU drops from ~42 % to ~8–15 %. Total 6-node sum drops from ~49 % to ~15–20 %.

**Decision:** If the delta is that large, add to README:
> For deployments where /utlidar/imu 250 Hz rate monitoring is not needed, remove it from `helix_topic_rate_monitor.topics` to reduce adapter CPU by ~30 percentage points.

**Risk:** Low. No code changes, just a param override on the adapter.

**Time:** ~10 min including relaunch + measure.

---

## Skip list (documented in Phase 2 summary, not worth further hardware time)

- GO2 mode-change anomaly scenarios (unsafe: requires physically inducing lidar dropout / sport transitions).
- Heartbeat from real monitored nodes (no such node exists — would need to write one, becomes a code-change task not a validation task).
- Cross-device DDS latency re-measurement (already covered in Session 1 at 0.81 ms, no reason to expect drift).

---

## Finishing artifacts

After running any of 1–3:

1. `rsync -a unitree@192.168.0.2:/home/unitree/helix_live_20260416_p2/ "T7:LABWORK/HELIX/hardware_eval_20260416_adapter_live/jetson_live_copy_phase3/"`
2. Append a "Phase 3 / 2026-04-XX" section to `HELIX PHASE 2 SUMMARY APR 16.md` OR create `HELIX PHASE 3 SUMMARY <date>.md` in this `HARDWARE RUNS/updates/` folder.
3. If the 1-hour run changes the RSS story, update §4C of the Phase 2 summary ("Resource trend over 20 min") accordingly.

## Doc-patch backlog (still not applied)

Queued from Phase 2 §11:

- `README.md`: delete passive_adapter.py paragraph; add packaged helix_adapter taxonomy; update old `<0.5% CPU / <50 MB` projection with real `~49–57 % of one core / ~260 MB RSS`.
- `docs/GO2_HARDWARE_EVIDENCE.md`: add Session 6 entry (2026-04-16) covering 20-min stability + 60 live ANOMALY faults + mode-dependent /diagnostics + sim_mode.
- `docs/GO2_ATTACHABILITY_UPDATE.md`: replace "2/4 native" with "1–2/4 native, mode-dependent."
- `docs/PAPER_CLAIMS.md`: can now claim all three fault-type paths validated end-to-end on hardware.

These doc patches should land in one narrow commit, gated on the 1-hour stability result.
