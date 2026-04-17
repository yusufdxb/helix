# HELIX Summary — 2026-04-16

Bounded hardware-validation pass of the current canonical main-branch adapter path on the live GO2 / Jetson Orin NX setup.

---

## 1. What was run

- `ros2 launch helix_bringup helix_sensing.launch.py` → 3 `helix_core` lifecycle nodes (heartbeat_monitor, anomaly_detector, log_parser) configured + activated.
- `ros2 launch helix_bringup helix_adapter.launch.py` → 3 `helix_adapter` lifecycle nodes (topic_rate_monitor, json_state_parser, pose_drift_monitor) configured + activated.
- 10 s `/helix/metrics` capture — 377 messages across 14 distinct metric labels.
- 60 s overhead sampling on all 6 helix PIDs + concurrent `/helix/faults` capture.
- Positive control: 3× `rcl_interfaces/msg/Log(level=ERROR, msg="Costmap failed to initialize — HELIX positive control N")` published on `/rosout`.
- Graceful `deactivate → cleanup` on all 6 nodes; Jetson process list clean at end.

## 2. On which machines

- **Jetson `unitree-jetson-payload`** (Orin NX, 4 physical cores, 15.4 GB RAM) — all ROS 2 work.
- **PC `careslab-OptiPlex-7050`** (Intel i7-7700 @ 3.60 GHz) — controller only, via SSH over CaresLab WiFi.
- **GO2** at `192.168.123.161` — live, stationary, reachable from Jetson at 0.17 ms ping.
- Source-of-truth repo: **`T7:LABWORK/HELIX/repo_clone`** on `main` at commit `26cca7d` (clean worktree). The local `/home/careslab/yusuf/helix` clone is stale (stops at `fe621e8`, has no `helix_adapter` package) — per lab convention, the T7 copy is canonical.
- Jetson workspace was synced from T7 (`rsync --delete` on `src/`), then `colcon build --symlink-install` (17.3 s, 4 packages clean).

## 3. Baseline (before any helix launch)

- `ros2 topic list` → **109 topics** (matches prior sessions).
- `ros2 node list` → empty. Jetson was ISOLATED: no come-here / phoenix / go2_bridge / behavior / audio / videohub / face / phoenix_lowcmd nodes running — Session 4 isolation preconditions satisfied.
- GO2 rate probe (5 s each, `ros2 topic hz`):

| Topic | Observed | Prior sessions |
|---|---|---|
| `/utlidar/imu` | 250.675 Hz | 250 Hz |
| `/utlidar/robot_odom` | 148.547 Hz | 150 Hz |
| `/utlidar/robot_pose` | 18.703 Hz | 18.7 Hz |
| `/utlidar/cloud` | 15.415 Hz | 15.4 Hz |
| `/gnss` | 1.000 Hz | 1 Hz |
| `/multiplestate` | 0.997 Hz | 1 Hz |

- Jetson baseline: thermal 44.3–48.6 °C, load `0.38 / 0.27 / 0.14`, 1.2 GB used / 12 GB free, 150 GB disk free.

## 4. Phase-by-phase results

### A — `helix_sensing.launch.py`

Nodes launched in `unconfigured [1]`; after `configure && activate`:

    /helix_heartbeat_monitor : active [3]
    /helix_anomaly_detector  : active [3]
    /helix_log_parser        : active [3]

Helix topics after activation:

    /helix/faults         Publisher=3 Subscription=0
    /helix/metrics        Publisher=0 Subscription=1   (anomaly_detector waiting)
    /helix/heartbeat
    /helix/node_health
    /helix_{heartbeat,anomaly,log_parser}/transition_event
    /diagnostics                                       (GO2 side)

### B — `helix_adapter.launch.py`

Adapter nodes launched `unconfigured [1]` → `configure` → `active [3]`:

    /helix_topic_rate_monitor : active [3]
    /helix_json_state_parser  : active [3]
    /helix_pose_drift_monitor : active [3]

`/helix/metrics` now `Publisher=3 Subscription=1` — canonical adapter → anomaly_detector link live on hardware.

### C — 10 s metric capture (live GO2 data)

`ros2 topic echo /helix/metrics` for ~10 s → 377 messages, ~27 Hz aggregate, 14 distinct `layout.dim[0].label` values:

| Metric | n | min | mean | max |
|---|---:|---:|---:|---:|
| rate_hz/utlidar_imu | 19 | 249.408 | **249.997** | 250.542 |
| rate_hz/utlidar_robot_odom | 19 | 146.918 | **148.853** | 150.471 |
| rate_hz/utlidar_robot_pose | 19 | 18.600 | **18.743** | 18.830 |
| rate_hz/utlidar_cloud | 19 | 15.402 | **15.408** | 15.414 |
| rate_hz/gnss | 19 | 0.999 | 0.999 | 0.999 |
| rate_hz/multiplestate | 19 | 0.996 | 0.997 | 0.997 |
| gnss/satellite_total | 18 | 13.000 | 13.000 | 13.000 |
| gnss/satellite_inuse | 1 | 0.000 | 0.000 | 0.000 |
| gnss/hdop | 19 | 0.000 | 0.000 | 0.000 |
| go2_state/brightness | 19 | 0.000 | 0.000 | 0.000 |
| go2_state/obstaclesAvoidSwitch | 19 | 1.000 | 1.000 | 1.000 |
| go2_state/uwbSwitch | 19 | 1.000 | 1.000 | 1.000 |
| go2_state/volume | 19 | 7.000 | 7.000 | 7.000 |
| pose/displacement_rate_m_s | 19 | 0.000 | 0.001 | 0.002 |

All three adapter subsystems (rate, JSON, pose-drift) publishing live-GO2-derived values. Rate-monitor numbers track the independent `ros2 topic hz` baseline within <1 %. Robot stationary → `pose/displacement_rate_m_s` sat near 0 (max 0.002 m/s noise floor). `/helix/faults` during same window: **0 events** (expected, no stimulus).

### D — 60 s overhead + stability

60 samples of `psutil.cpu_percent(None)` + `memory_info().rss` on all 6 helix PIDs:

| Node | CPU % (mean / max) | RSS MB (mean / max) |
|---|---:|---:|
| helix_heartbeat_monitor | 0.60 / 2.0 | 36.84 / 37.02 |
| helix_anomaly_detector | 2.28 / 3.0 | 35.49 / 35.67 |
| helix_log_parser | 0.37 / 1.0 | 36.54 / 36.73 |
| helix_topic_rate_monitor | **42.03** / **45.9** | 59.94 / 60.16 |
| helix_json_state_parser | 1.07 / 2.0 | 37.27 / 37.46 |
| helix_pose_drift_monitor | 2.53 / 4.0 | 50.70 / 50.93 |
| **Total (6 nodes)** | **48.88** | **256.78** |

Jetson host load during run: `1.43 / 0.68 / 0.36`. Thermal peak 51.4 °C (+3–5 °C from baseline). **All 6 nodes remained `active` throughout** — lifecycle stable for the bounded window. Fault events emitted during 60 s: **0**.

### E — Positive control (`/rosout` pattern)

Published 3× ERROR `Log` messages matching `log_rules.yaml` rule `nav2_costmap_fail` (regex `Costmap.*failed to initialize`). Observed `/helix/faults`:

    node_name: helix_pc_test
    fault_type: LOG_PATTERN
    severity: 3
    detail: 'Nav2 costmap initialization failure detected | Log: Costmap failed to initialize — HELIX positive control 0'
    context_values: [nav2_costmap_fail, 40, helix_pc_test, positive_control.py]

**1 FaultEvent emitted for 3 injected logs** — consistent with `dedup_window_sec=5.0` suppressing the two later matches. End-to-end `LOG_PATTERN` path verified on hardware (pub → log_parser subscription → regex match → dedup → FaultEvent pub).

### F — Graceful shutdown

All 6 nodes transitioned `active → deactivate → cleanup` successfully, then processes killed. Jetson `ps | grep helix` empty at end.

## 5. What this run PROVES

1. **Canonical main-branch launch path works on hardware.** `ros2 launch helix_bringup helix_sensing.launch.py` + `ros2 launch helix_bringup helix_adapter.launch.py` bring 6 lifecycle nodes to `active` on the Jetson on current commit `26cca7d`.
2. **Packaged `helix_adapter` publishes real GO2-derived metrics** — topic-rate, JSON-state, and pose-drift subsystems observed live with sane values matching the independent baseline rate probe within <1 %.
3. **End-to-end LOG_PATTERN detection works on hardware.** `/rosout` injection → log_parser regex match → FaultEvent on `/helix/faults`, with dedup working (3 in, 1 out within 5 s).
4. **Lifecycle stable across ~6 min of active operation** — all 6 nodes remained `active` through metric capture, overhead sampling, PC injection, and final deactivate.

## 6. What this run does NOT prove

- **Persistent deployment.** ~6 min is not a long-duration test. Session 4 already flagged instability of 30-min runs under contention; this run was bounded and isolated. No claim about continuous monitoring.
- **Anomaly detector path on live GO2 data.** `/helix/faults` saw 0 non-injected events during the 90 s+ of active capture. Under stationary baseline, no natural anomaly occurred. No artificial rate perturbation was applied because safe stimulation of GO2-published rates isn't trivial without disturbing the robot. This is silence in the absence of stimulus, NOT proof that anomaly detection works on live GO2 rates.
- **Heartbeat topic content.** `/helix/heartbeat` capture window was empty due to a concurrent-echo race; heartbeat_monitor process did reach `active` and consumed normal CPU/RSS, but the topic sample is not in this session's evidence. (`/helix/node_health` capture at 1 Hz did produce messages, confirming the node's diagnostic aggregator side was alive.)
- **Attachability matrix freshness.** `docs/GO2_ATTACHABILITY_UPDATE.md` was not re-run; assumed unchanged from 2026-04-06 baseline.

## 7. Overhead claim vs. prior README framing

The historical README states for hardware evaluation:
> HELIX resource estimates (<0.5% CPU, <50 MB RAM, +0.2°C) are **projected** from PC measurements and algorithmic scaling factors.

Fresh measurement on the canonical adapter path contradicts this projection **when the adapter is included**:

- Total CPU: **~49 % of one Jetson core** (~12 % of total 4-core capacity).
- Total RSS: **~257 MB** vs. <50 MB projected.
- Dominated by `helix_topic_rate_monitor` (~42 % of one core), which has Python callbacks at ~440 Hz aggregate (IMU 250 + odom 148 + pose 18.7 + cloud 15.4 + 1 + 1 Hz).

The old <0.5 % / <50 MB numbers were for `helix_core` in isolation, not the `helix_core + helix_adapter` canonical path.

## 8. Docs-vs-current-state drift (NOT auto-fixed this run)

- `README.md` still describes `scripts/passive_adapter.py` in the Research Context section. That script was deleted in commit `26cca7d`. The README has not been updated to describe the packaged `helix_adapter` path. Recommend a narrow doc patch once a longer bounded run is recorded.
- `docs/GO2_HARDWARE_EVIDENCE.md` only covers Sessions 1 (2026-04-03) and 2 (2026-04-06). Sessions 3–5 and this 2026-04-16 run are not yet merged into the in-tree doc.
- README "Hardware Evaluation Demonstrated" bullets still say "PC observing the GO2 graph" — this run proves the Jetson-side canonical path, which the doc does not yet mention.

## 9. Honest claim boundaries for repo docs

| Claim | Status |
|---|---|
| Canonical main-branch hardware path validated | **Yes, bounded** — Jetson, ~6 min, `26cca7d` |
| Canonical adapter path validated on live GO2 data | **Yes for publish path** (14 labels with real values). **Not validated for anomaly-detection output** on live GO2 rates. |
| Current docs match current hardware evidence | **No** — README still describes deleted `scripts/passive_adapter.py`; `docs/GO2_HARDWARE_EVIDENCE.md` stops at Session 2. Docs NOT auto-edited this run. |

## 10. Artifact paths (on T7)

- **Session root:** `/media/careslab/T7 Storage/LABWORK/HELIX/hardware_eval_20260416_adapter_live/` (5.4 MB)
- **Session notes (full detail):** `…/notes/SESSION_NOTES.md`
- **Provenance:** `…/env/{pc,jetson}_provenance.txt`
- **Jetson working-copy mirror:** `…/jetson_live_copy/{env,logs,results}/`
- **Authoritative overhead JSON:** `…/jetson_live_copy/results/overhead_60s_v2.json`
- **Positive-control fault:** `…/jetson_live_copy/results/faults_during_pc.txt`
- **Metric samples:** `…/jetson_live_copy/results/metrics_full_10s.txt` (raw), `metrics_summary.txt` (parsed table), `metrics_labels_unique.txt`
- **Lifecycle traces:** `…/jetson_live_copy/logs/{sensing,adapter}_lifecycle_{before_config,transitions,after_activate}.txt` + `…/results/lifecycle_{final,shutdown,after_overhead_run}.txt`
- **Backup of pre-run Jetson src:** `Jetson:~/helix_ws_backups/src_before_20260416_220426.tar.gz` (21 KB)

## 11. Reproduction

On a Jetson at `unitree@192.168.0.2` with `T7:LABWORK/HELIX/repo_clone` rsynced to `~/helix_ws/src/` and colcon-built, source the env helper and:

    ros2 launch helix_bringup helix_sensing.launch.py &
    sleep 3
    for n in /helix_heartbeat_monitor /helix_anomaly_detector /helix_log_parser; do
      ros2 lifecycle set $n configure; ros2 lifecycle set $n activate; done
    ros2 launch helix_bringup helix_adapter.launch.py &
    sleep 3
    for n in /helix_topic_rate_monitor /helix_json_state_parser /helix_pose_drift_monitor; do
      ros2 lifecycle set $n configure; ros2 lifecycle set $n activate; done

Positive control:

    python3 -c "
    import rclpy, time
    from rcl_interfaces.msg import Log
    rclpy.init(); n=rclpy.create_node('pc'); p=n.create_publisher(Log,'/rosout',10); time.sleep(1)
    for i in range(3):
      m=Log(); m.level=40; m.name='pc'; m.msg=f'Costmap failed to initialize test {i}'; m.file='a.py'; p.publish(m); time.sleep(0.3)
    time.sleep(1); rclpy.shutdown()"

Env helper used on this run:

    source /opt/ros/humble/setup.bash
    source /home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash
    source /home/unitree/helix_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
    export ROS_DOMAIN_ID=0
