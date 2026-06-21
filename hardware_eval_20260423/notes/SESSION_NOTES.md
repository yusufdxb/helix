# HELIX Session 8 — 2026-04-23

**Lab card:** `LABWORK/HELIX/LAB_CARD_2026-04-22.md`
**Host:** lab PC `ds027827` (cares@)
**Target main HEAD:** `4c43778`
**T7 path:** `/media/cares/T7 Storage/LABWORK/HELIX/hardware_eval_20260423/`

## Step 1 — Jetson sync + rebuild

### State at session open

- Lab PC T7 `repo_clone` HEAD: `4c43778` clean ✓
- Jetson pre-sync HEAD: `92cbbcc` (not `fe621e8` as card said — someone touched it)
- Jetson working tree had ` M RESULTS.md`, ` M benchmark_results.json`
  (auto-generated benchmark outputs — Apr 15 mtime)
- Jetson has no internet (`github.com` DNS fails on CaresLab LAN)

### Actions

1. Jetson stash: `git stash push -m 'pre-sync-2026-04-23 ...' -- RESULTS.md benchmark_results.json`.
   Saved as `stash@{0}`.
2. Backup patch written to `logs/jetson_prestash_RESULTS_benchmark.patch`
   (98 lines, so the diff is recoverable even if the stash is lost).
3. On Jetson: `rm -rf build install log` (per card).
4. rsync from T7 `repo_clone/` → `unitree@192.168.0.2:/home/unitree/yusuf/helix/`
   with `--delete`, excluding `build/install/log/` and `hardware_eval_*/bags/`
   to preserve Jetson-side bag artifacts and avoid recopying giant bag files.
   Transferred 335 files (~4.7 MB literal data).
5. Jetson HEAD post-rsync: `4c43778` ✓

### Build

Kicked off on Jetson (backgrounded):

```
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  helix_msgs helix_core helix_adapter helix_bringup \
  helix_diagnosis helix_recovery helix_explanation \
  > /tmp/helix_colcon_build.log 2>&1
```

Build log copied to `logs/jetson_colcon_build.log`. Result: **EXIT_0, 7 packages, 22.8 s**.

## Step 2 — SENSE + ADAPTER bringup — DONE

- DDS env: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
  `CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml`.
  Helper script written to `/home/unitree/yusuf/helix/jetson_env.sh`.
- `ros2 topic list` → 109 topics (matches Session 6). `/sportmodestate` ✓, `/utlidar/cloud` ✓.
- Launched via `setsid bash -c '... exec ros2 launch ...' & disown`:
  - `helix_sensing.launch.py`, log `logs_20260423/sensing_launch.log`
  - `helix_adapter.launch.py`, log `logs_20260423/adapter_launch.log`
- All 6 lifecycle nodes `active [3]`:
  `helix_heartbeat_monitor`, `helix_anomaly_detector`, `helix_log_parser`,
  `helix_topic_rate_monitor`, `helix_json_state_parser`, `helix_pose_drift_monitor`.

## Step 3 — DIAGNOSE + RECOVER + EXPLAIN bringup — DONE (recovery intentionally held `unconfigured`)

Nodes started via `start_helix_closedloop.sh` on Jetson:
- `helix_context_buffer`   (lifecycle) — **active**
- `helix_diagnosis_node`   (lifecycle) — **active**
- `helix_recovery_node`    (lifecycle) — **unconfigured** ← held here deliberately
- `helix_llm_explainer`    (regular Node, `llm_enabled=false`) — **alive**, template-only

Sensor rates going in:
- `/sportmodestate` ≈ 295 Hz
- `/utlidar/cloud` ≈ 15.4 Hz

No fault events in `/helix/faults` yet (expected, topics flowing normally).

Recovery was launched with default `enabled=false`. Plan: before step 4, kill + relaunch
with `--ros-args -p enabled:=true` and cycle to active. The allowlist is hard-coded to
`{STOP_AND_HOLD, RESUME, LOG_ONLY}` in `recovery_node.py`; the card's "allowlist
{STOP_AND_HOLD, LOG_ONLY}" is operational (don't inject scenarios that produce RESUME)
not a config switch. For this demo: inject LiDAR cover only → STOP_AND_HOLD path only.

## Steps 4–7 — DONE with a bug fix in the middle

**Short version:** closed loop validated end-to-end on live GO2 hardware
after fixing a schema mismatch in R1 that made diagnosis never fire.
Full writeup: `results/closed_loop_demo.md`.

### Step 4 — recovery enabled

- Killed initial `helix_recovery_node` (launched with default `enabled=false`).
- Relaunched with `--ros-args -p enabled:=true -p cooldown_seconds:=5.0`.
- Lifecycle configure + activate → `active [3]`, `enabled=True`.

### Step 5 — operator safety gate

- Operator at robot, e-stop ready.
- Live `/sportmodestate.body_height` = **0.315 m** (threshold 0.25 m ✓).
- `mode = 0` (standing idle).

### Step 6 — first injection attempt (UDP drop)

- Discovered `/helix/cmd_vel` has 0 subscribers — robot isn't wired to our
  actuation path yet. Zero actuation risk. Proceeded with UDP drop as
  safer alternative to "physical LiDAR cover" (which doesn't affect Livox
  publish rate anyway — confirmed experimentally).
- `iptables -I INPUT -i enP8p1s0 -p udp -j DROP` held ~44 s.
- Bag `bags/lidar_occlusion_demo/` (638 MB) — 12 ANOMALYs fired, but
  **all before** the drop (natural jitter). Zero during/after the drop
  because `topic_rate_monitor` emits NaN for stale topics and
  `anomaly_detector` rejects NaN. **Documented product gap.**
- 12 faults → **0 hints → 0 cmd_vel.** Closed loop broken at diagnosis.

### Bug discovered: R1 schema mismatch

- `_emit_anomaly_fault` writes `context_keys=['metric_name', ...]` with
  values `rate_hz/utlidar_*`; FaultEvent has no `fault_id` field.
- `_rule_r1` looks for key `'metric'` with literal `'utlidar_rate'` and
  dereferences `fault.fault_id`.
- R1 never matched any live ANOMALY (would AttributeError if it had).

### Fix applied on branch `fix/r1-anomaly-schema-mismatch`

- `rules.py`: `_metric_name` accepts both keys; R1 matches `rate_hz/utlidar*`
  prefix; `_fault_source_id(fault)` returns `node_name`.
- `test_rules.py`, `test_diagnosis_node.py`: fixtures updated to match
  live schema.
- **16/16 tests pass on Jetson** (pytest with real helix_msgs).
- Committed on T7 at `e821b47`. `main` untouched.
- Rsync'd to Jetson → `colcon build --packages-select helix_diagnosis`
  (2.5 s, clean). Diagnosis node restarted, activated.

### Step 7 — retry demo (post-fix)

- Fresh bag `bags/post_fix_demo/` (75 MB, 7m19s).

| Topic | Count |
|---|---|
| `/helix/faults` | 30 (ANOMALY on utlidar_robot_odom, utlidar_imu, gnss, multiplestate) |
| `/helix/recovery_hints` | **14** (9 × R1 STOP_AND_HOLD utlidar-only, 5 × R2 RESUME) |
| `/helix/recovery_actions` | 14 audit entries (7 ACCEPTED, 7 SUPPRESSED_COOLDOWN) |
| `/helix/cmd_vel` | **3064** zero-twist @20 Hz during STOP states |

Pipeline proven: SENSE → DIAGNOSIS → RECOVERY → cmd_vel on live hardware.
Safety envelope (allowlist + cooldown) visibly working in audit log.

### Bag copied to T7: `bags/post_fix_demo/post_fix_demo_0.db3`

## Pending steps (none for today)

Previous list complete. Follow-ups in `results/closed_loop_demo.md`.
- [ ] Step 5 — **OPERATOR SAFETY GATE** — GO2 standing, `body_height > 0.25 m`, e-stop ready
- [ ] Step 6 — inject `/utlidar/cloud` occlusion
- [ ] Step 7 — capture bag + screen record as demo video

## Hard rules (carry-forward)

- No "validated" claims without artifacts under this `hardware_eval_` dir.
- Raw bags stay on T7; commit only summaries + the key demo bag.
- No `Co-Authored-By: Claude` in commits.
- Before pose-changing sport commands: verify `body_height` from `/sportmodestate`.
- `motion_switcher` SelectMode: `"mcf"` or `"ai"` only — never `"normal"`.
