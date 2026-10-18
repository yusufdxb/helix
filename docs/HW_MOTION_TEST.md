# GO2 hardware test: HELIX STOP reaches the motors

One bounded session. Six stages, each run by one command that runs that stage
only and exits. **No stage starts the next one.** `helix_hw_stage` refuses a
stage unless:

- the previous stage's evidence is PASS, with the same git SHA and config hash;
- that stage's preflight is GO;
- the operator types the stage's confirmation phrase.

A PASS is never overwritten. A FAIL can be retried, and the failed evidence is
archived next to the retry. A rehearsal session can never unlock a hardware
stage.

Design and off-robot results are in [`MOTION_ARBITRATION.md`](MOTION_ARBITRATION.md).
Physical stopping is **not claimed** until stage E passes on the robot.

| Stage | What runs | Sink mode | Robot | Pass criteria (all must pass) |
|---|---|---|---|---|
| A | Graph and topics, nothing commanded | `dry_run` | motors off / damped, lying down | preflight GO; 5 s zero output; no Move; robot still |
| B | Live arbitration, **zero only** | `stop_only` | standing (mcf), remote in hand | arbiter selects nav at 0.0; StopMove reaches the robot; **robot answers code 0**; no Move; robot still |
| C | 0.10 m/s through the arbiter, then injected fault | `dry_run` (nothing reaches the robot) | physically staged | 0.10 visible on `/cmd_vel`; sink *decides* Move (not sent); full fault chain to zero; sink decides StopMove(ZERO); robot still |
| D | One bounded move: 0.15 m/s for 2.0 s, then zero | `armed` | standing, 2 m clear, spotter | peak 0.05 to 0.25 m/s; StopMove sent after zero; stopped < 1.5 s |
| E | 0.15 m/s; benign fault injected mid-motion; **upstream keeps streaming** | `armed` | as D | moving > 0.05 m/s at injection; full chain fault, hint, action, hold, arbiter zero, output zero; 0 nonzero outputs while held; StopMove reached and acknowledged by the robot; **no Move sent while held**; **robot physically stopped < 1.5 s after the hold**; RESUME; no motion after RESUME |
| F | Release and operator re-arm, no upstream command | `armed` | as D | hold, then RESUME, then 5 s still; recovery deactivate forces zero; reactivate; 5 s still; arbiter ends in NO_LIVE_INPUT; no Move |

Every stage also fails if HELIX raises a **real** (uninjected) STOP during it
("confounded"), or if the robot exceeds 0.35 m/s. On overspeed the runner
stops sourcing commands and publishes zero at once.

The injected fault is one `helix_msgs/FaultEvent` (ANOMALY, severity 2) with
`node_name` `rate_hz/utlidar_helix_injected`. It matches diagnosis rule R1,
touches no sensor, and cannot be confused with a real metric.

## Evidence

Every stage writes `<session>/stage_<X>/`:

- `evidence.json`: git SHA and dirty flag, config hash, live params of the
  arbiter, recovery and sink, sink mode, ROS graph, preflight checks,
  commanded inputs, every FaultEvent (injected and not), RecoveryHint,
  RecoveryAction, hold transitions, arbiter selections, sink decisions,
  measurements, timestamps, each check, and a verdict of PASS, FAIL or
  INCOMPLETE.
- `trace.jsonl`: every message plus robot odometry.
- `preflight.json`: the full graph snapshot.

A stage that was refused before running goes to `<session>/attempts/`.
The config hash covers `arbiter.yaml`, `helix_params.yaml`, the closed-loop
launch file, and the live parameters of arbiter, recovery and sink (sink
`mode` excluded).

Hardware evidence requires a clean git tree.

## Before the session (off-robot, mandatory)

The lab network has no internet. Build and stage everything in advance
(GO2_FIELD_NOTES.md section 7).

```bash
# on the workstation, at the candidate SHA
cd ~/workspace/helix && git checkout <CANDIDATE_SHA> && git status --porcelain   # must be empty
colcon build --symlink-install
./scripts/hw_rehearsal.sh            # must print STAGE A..F: PASS [REHEARSAL...]
```

Copy the same SHA to the payload and build it there, with `unitree_api`
available (unitree_ros2 `cyclonedds_ws`).

## Environment on the payload (every terminal)

```bash
source /opt/ros/humble/setup.bash
source <UNITREE_WS>/install/setup.bash          # provides unitree_api
source <HELIX>/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
grep -o 'NetworkInterface name="[^"]*"' "${CYCLONEDDS_URI#file://}"   # must be enP8p1s0
ip -brief link show enP8p1s0                                          # must be UP
sudo date -s "<current UTC time>"   # payload has no RTC; preflight C10 fails on a 1970 clock
ros2 daemon stop
ros2 topic hz /lowstate             # ~500 Hz, or stop here (topic presence is not data)
SESSION=~/helix_hw/$(date +%Y%m%d)_motion
```

## Terminals

```bash
# T1: HELIX stack + arbiter (recovery enabled; it can only hold, never move)
ros2 launch helix_bringup helix_closedloop.launch.py \
    auto_activate_recovery:=true recovery_enabled:=true

# T2: sink; Ctrl-C and relaunch with the mode each stage requires.
#     Ctrl-C sends a StopMove burst in stop_only/armed.
ros2 run helix_arbiter helix_go2_sport_sink --ros-args -p mode:=dry_run

# T3: optional independent bag
ros2 bag record -o $SESSION/bag /helix/faults /helix/recovery_hints \
    /helix/recovery_actions /helix/hold /helix/arbiter/status /cmd_vel /nav/cmd_vel \
    /helix/sink/trace /api/sport/request /api/sport/response /utlidar/robot_odom

# T4: stage runner (one stage per command)
```

Nothing else may publish `/cmd_vel`, `/nav/cmd_vel`, `/teleop/cmd_vel` or
`/helix/hold`. Preflight enforces this.

## Stages (T4)

Stand the robot only with the handheld remote (stand lock, then Start).
Never stand it through the API. Only `mcf` mode. Never `SelectMode('normal')`.

```bash
# A: motors off / damped, lying down. T2 mode:=dry_run
ros2 run helix_arbiter helix_hw_stage --stage A --session-dir $SESSION --repo <HELIX>
#   confirm phrase: MOTORS OFF
#   also writes $SESSION/sport_baseline.json (stock /api/sport/request publishers)

# B: operator stands the robot (mcf). T2: Ctrl-C, relaunch mode:=stop_only
ros2 run helix_arbiter helix_hw_stage --stage B --session-dir $SESSION --repo <HELIX>
#   confirm phrase: ZERO ONLY

# C: robot physically staged (sitting, held, or feet off the ground). T2 mode:=dry_run
ros2 run helix_arbiter helix_hw_stage --stage C --session-dir $SESSION --repo <HELIX>
#   confirm phrase: ROBOT STAGED

# D: robot standing, 2 m clear ahead, spotter. T2 mode:=armed
ros2 run helix_arbiter helix_hw_stage --stage D --session-dir $SESSION --repo <HELIX>
#   confirm phrase: AREA CLEAR MOVE

# E: as D, sink stays armed
ros2 run helix_arbiter helix_hw_stage --stage E --session-dir $SESSION --repo <HELIX>
#   confirm phrase: AREA CLEAR FAULT

# F: as D, sink stays armed. The runner pauses twice for the operator re-arm.
ros2 run helix_arbiter helix_hw_stage --stage F --session-dir $SESSION --repo <HELIX>
#   confirm phrase: OPERATOR REARM
```

## Dry variants of A and C (remapped topics)

Stages A and C run the sink in `dry_run`, so they can also run with every
motion topic moved onto sink topics that nothing on the robot consumes. Use
this to check the graph, the fault chain and the STOP decisions on the robot
without the arbiter ever publishing `/cmd_vel` or the runner publishing
`/nav/cmd_vel`. The defaults are unchanged: without the options below the
runner uses the real topics exactly as in the stages above.

| Topic | Real (default) | Dry (`--topic-prefix`) |
|---|---|---|
| arbiter output | `/cmd_vel` | `/helix_dry/cmd_vel` |
| nav source (the runner publishes here) | `/nav/cmd_vel` | `/helix_dry/nav/cmd_vel` |
| teleop source | `/teleop/cmd_vel` | `/helix_dry/teleop/cmd_vel` |

```bash
DRY=~/helix_hw/$(date +%Y%m%d)_dry        # its own session dir, never $SESSION
SHARE=$(ros2 pkg prefix helix_arbiter)/share/helix_arbiter

# T1: arbiter on the dry topics
ros2 launch helix_bringup helix_closedloop.launch.py \
    auto_activate_recovery:=true recovery_enabled:=true enable_twist_mux:=false \
    arbiter_config:=$SHARE/config/arbiter_dry.yaml cmd_vel_out:=/helix_dry/cmd_vel

# T2: sink, dry_run only, reading the dry output
ros2 run helix_arbiter helix_go2_sport_sink --ros-args -p mode:=dry_run \
    -p input_topic:=/helix_dry/cmd_vel

# T4
ros2 run helix_arbiter helix_hw_stage --stage A --session-dir $DRY --repo <HELIX> --topic-prefix
ros2 run helix_arbiter helix_hw_stage --stage C --session-dir $DRY --repo <HELIX> --topic-prefix
```

`--topic-prefix` alone means `/helix_dry`; `--topic-prefix /other` or
`--cmd-topic`, `--nav-topic` and `--teleop-topic` (all three) choose other
names. The standalone `helix_preflight` takes the same options.

What keeps a dry PASS from standing in for a real one:

- Only stages A and C accept remapped topics; B, D, E and F refuse them.
- A remap must move cmd, nav and teleop all off the real path. A partial remap,
  or a remapped name that is itself a robot command topic (`/cmd_vel`,
  `/nav/cmd_vel`, `/teleop/cmd_vel`, `/helix/cmd_vel`, `/api/sport/request`,
  `/lowcmd`, `/wirelesscontroller`), is rejected before anything starts.
- Evidence records the topic set as `"topics": {"mode": "remapped", ...}`
  (real runs record `"mode": "real"`), and the summary line reads
  `STAGE A: PASS [TOPICS REMAPPED: NOT REAL COMMAND-PATH EVIDENCE]`.
- The session dir gets a `TOPICS_REMAPPED` marker. Real stages refuse to run
  in it, and dry stages refuse a dir that already holds real evidence.
- Dry stages chain A then C. Every real stage requires real-topic evidence
  from its predecessor, so remapped evidence never unlocks stage B or later,
  even if copied into a hardware session.
- Preflight adds C14: no HELIX node publishes or subscribes any of the real
  command topics listed above.
- The live arbiter parameters differ (`output_topic`, sources), so the config
  hash differs from a real run as well.

A dry stage A or C PASS says the HELIX chain works on the robot's compute with
the real sensors and clocks. It says nothing about the real `/cmd_vel` edge,
the sink's `/api/sport/request` path or the robot's response; only the real
stages cover those.

Stand-alone preflight at any time (publishes nothing):

```bash
ros2 run helix_arbiter helix_preflight --stage E --sport-baseline $SESSION/sport_baseline.json
```

## GO / NO-GO (preflight, every stage)

Any FAIL is NO-GO and the stage does not run.

| ID | Condition | Notes |
|---|---|---|
| C1 | `/helix_arbiter`, `/helix_recovery_node`, `/helix_diagnosis_node`, `/helix_go2_sport_sink` present | |
| C2 | arbiter, recovery, diagnosis lifecycle `active` | |
| C3 | types: `/cmd_vel` Twist, `/helix/hold` HelixHold, `/helix/arbiter/status` ArbiterStatus, `/utlidar/robot_odom` Odometry, sources Twist, `/api/sport/request` unitree_api Request (stages B, D, E, F) | |
| C4 | QoS compatible on every motion edge (reliability, durability) | |
| C5 | `/cmd_vel` has exactly one publisher: `/helix_arbiter` | |
| C6 | `/cmd_vel` has exactly one consumer: `/helix_go2_sport_sink` | recorders and HELIX observers excluded |
| C7 | sink mode matches the stage (A, C dry_run; B stop_only; D, E, F armed) | |
| C8 | no competing motion authority: no `twist_mux`; `/helix/cmd_vel` unused; every source topic read by the arbiter; exactly the sink publishing `/api/sport/request` from HELIX (none in dry_run); no `/api/sport/request` publisher outside the stage-A baseline | |
| C8b | stage-A baseline supplied (B to F) | |
| C9 | `/utlidar/robot_odom` >= 50 Hz and age <= 0.2 s | WARN only in A (motors off) |
| C10 | local clock after the HEAD commit time and year >= 2026 | |
| C10b | robot header clock skew recorded | WARN only; freshness never uses robot stamps |
| C11 | HELIX hold state fresh (<= 0.5 s) | |
| C11b | HELIX **not holding** at stage start | WARN only in A |
| C12 | arbiter output zero at preflight | |
| C13 | `/helix/hold` published only by recovery and `/helix/recovery_hints` only by diagnosis | a second publisher could release a hold |
| C14 | dry (remapped) runs only: no HELIX node on any real command topic | see Dry variants |

## Abort

- Handheld remote or the lab e-stop, at any time. They sit outside this path.
- Ctrl-C in T4. The runner publishes zero on `/nav/cmd_vel` on the way out,
  which the arbiter passes straight through, and the sink sends StopMove. If
  that zero were lost, the source timeout would force zero within 0.5 s.
- Ctrl-C in T2 sends a StopMove burst.
- Ctrl-C in T1 (arbiter) publishes a zero burst. If the arbiter dies without
  one, the sink's deadman sends StopMove after 0.25 s.

After any abort, the session continues only from a fresh stage run. PASS
evidence stays valid only while the SHA and config hash are unchanged.
