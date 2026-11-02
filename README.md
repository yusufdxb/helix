# HELIX

**A small self-healing supervisor for ROS 2: it finds which node failed and why, applies a bounded repair, checks that the outputs came back, and falls back safely when they do not.**

[![CI](https://github.com/yusufdxb/helix/actions/workflows/ci.yml/badge.svg)](https://github.com/yusufdxb/helix/actions/workflows/ci.yml)
![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E)
![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C)
![License: MIT](https://img.shields.io/badge/license-MIT-green)

`respawn=true` restarts a node when its process exits. A typical watchdog also restarts a node whose output went
stale. Neither one can tell a node that hung from a node starved by a dead upstream. Neither notices a node that still
publishes, but at a third of its rate. Neither knows that restarting into a crash loop or a QoS misconfiguration
achieves nothing. HELIX (`helix_healer`) is one C++20 process, about 1,500 lines, that launches and supervises the
nodes of a pipeline from a plain-text spec. Every 50 ms it runs one loop:

1. **Observe.** Nothing is added inside the managed nodes. HELIX reads:
   - process exit status (`waitpid`)
   - CPU and RSS from `/proc`
   - the age and rate of every declared output
   - whether each output is still advertised, and any QoS-incompatible events
   - the upstream graph
2. **Diagnose** each node as one of: crash, crash loop, memory leak, CPU runaway, QoS mismatch, endpoint loss,
   hang, degraded rate, or *upstream*. Upstream means the node is a victim, so HELIX leaves it alone.
3. **Repair** from a closed set:
   - restart with backoff
   - lifecycle reset through the node's own lifecycle services
   - start a declared backup node
   - report only

   The budget is 3 attempts per 60 s.
4. **Verify** that the node is alive and its outputs are healthy within 5 s. A failed lifecycle reset escalates to a
   restart.
5. **Fall back.** When the budget is spent, or the fault cannot be fixed by a restart (a crash loop, a QoS mismatch),
   HELIX starts the backup node if there is one. Otherwise it publishes a latched `SAFE_STOP <node>` on `/helix/mode`.

Stopping is the last resort, not the feature. HELIX has no publisher on any command topic, and it runs only the
`exec` lines in its spec. Every decision goes to a JSONL event log and to `/helix/heal_events`. Per-node state goes to
`/diagnostics`. The full method is in [docs/v3/method.md](docs/v3/method.md).

## Results (held-out, pre-registered)

The healer was developed on one pipeline (a navigation stack, `nav.spec`). It was then frozen and run once on a
second pipeline it had never seen (a manipulation stack, `manip.spec`). That pipeline has a lifecycle camera with a
backup, a detector that latches when its input disappears, a tracker, an arm planner, a controller and a gripper.
The claim and its kill criterion were registered before the held-out run
([preregistration](docs/preregistration/2026-11-02-helix-v3-heldout-manip.md)). The full write-up, with the audit
and every deviation, is in [docs/v3/results/heldout_manip.md](docs/v3/results/heldout_manip.md).

The run covered 14 fault scenarios plus nominal, 3 reps each, 45 episodes per policy. Faults were injected 7 s into
each episode. All four policies are the same binary and differ only in `policy:=`. The outcome is judged by an
independent probe, not by the healer: RESTORED means every output is at least 0.8 x nominal rate for the whole
final 3 s.

| Policy | Restored | Safe stop | Failed | Root diagnosis correct | Unnecessary interventions | Mean outage |
|---|---|---|---|---|---|---|
| none (observe only) | 9 | 0 | 36 | n/a | 0 | 21.1 s |
| respawn (`respawn=true`) | 15 | 0 | 30 | 6/42 | 0 (141 futile) | 17.7 s |
| watchdog (exit, stale or RSS; same budget, verify and fallback) | 33 | 6 | 6 | 18/42 | 48 | 9.1 s |
| **HELIX** | **39** | **6** | **0** | **42/42** | **0** | **5.7 s** |

The 6 safe stops are the two faults no repair can fix: a crash loop with no backup and a QoS mismatch. Both HELIX
and the watchdog end in SAFE_STOP there. HELIX recognises the crash loop and stops 1.65 s after the fault.

- **The registered claim survived.** HELIX had to restore at least 3 more episodes than the watchdog, with no more
  unnecessary interventions. It restored 6 more, with 0 unnecessary interventions against the watchdog's 48.
- **The 6 extra episodes come from rate degradation.** All of them are in `degrade` and `degrade_lifecycle`, where a
  node keeps publishing at a fraction of its rate. A node like that is never stale, so the watchdog never acts.
- **The watchdog's extra interventions do not cost it episodes.** It restarts starved downstream nodes 48 times. That
  does not reduce its restored count, but its mean outage is 9.1 s against 5.7 s.
- **HELIX used half the watchdog's interventions.** It made 60 decisions to the watchdog's 120. It also left no
  runaway process burning CPU, where the watchdog left 3.
- **The result is not statistically significant.** At the registered unit, the scenario, only 2 of 14 blocks differ.
  The exact sign-flip p is 0.25, which is the floor for 2 blocks. The development pipeline gave the same pattern:
  +6 restored and 0 against 51 unnecessary interventions.
- **The healer costs 0.7% of one core and 15 MB RSS.**

### Learned diagnosis: tested, not adopted

The question was whether a model beats the deterministic rules at diagnosis on held-out faults. The models output
only a (node, fault) label from closed enums and never choose or execute actions.

| Diagnoser | Correct at the decision point (n=42) | False alarms (fault-free) |
|---|---|---|
| HELIX rules | **42/42** | 0 |
| Gradient-boosted trees, trained on the dev pipeline only | 36/42 | 2 / 3504 snapshots |
| qwen3 4B instruct, local GPU, JSON-schema constrained | 21/42 | not measured |
| qwen3 8B, local GPU, JSON-schema constrained | 19/42 | not measured |

The integration rule was registered in advance: a learned diagnoser is adopted only if it beats the rules by at
least 2 with no more false alarms. No arm met it, so the rules stay.

The trees generalised to the unseen topology on most faults and detected symptom faults faster than the rules:

| Fault | Trees | Rules |
|---|---|---|
| leak | 1.5 s | 3.4 to 3.9 s |
| CPU runaway | 0.9 s | 2.4 s |

They missed every hang on two of the nodes. Both LLMs mostly labelled hangs as "degraded". A hybrid, with the
rules for exits and a learned detector for slow symptoms, is untested.

## What is not shown

- **Synthetic faults on stand-in nodes.** The managed nodes are `helix_demo/demo_node` stages that inject faults on
  command:
  - segfault and exit
  - hang
  - endpoint loss
  - CPU hog
  - leak
  - rate degradation
  - QoS mismatch
  - missing device

  They are not real drivers, planners or perception stacks.
- **DDS disruption is only partly covered.** It is represented by endpoint loss and QoS mismatch. There are no
  network partitions, discovery storms or RMW failures.
- **One machine, two topologies, 3 reps.** The effect rests on 2 scenario blocks, and the result is not significant.
- **No hardware.** Nothing here ran on a robot. Actuation was never enabled.
- **The healer must own the processes.** It launches the nodes itself, so it does not attach to nodes started
  elsewhere, and it cannot repair the composable nodes inside a shared container one at a time.

## Quick start

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to helix_healer helix_demo
source install/setup.bash
ros2 launch helix_healer healer.launch.py spec:=src/helix_healer/config/manip.spec policy:=helix event_log:=/tmp/heal.jsonl
ros2 topic echo /helix/heal_events     # one line per diagnosis, repair, verification, give-up
ros2 topic echo /helix/mode            # latched; SAFE_STOP <node> only when every repair is exhausted
```

A spec declares each node, what it publishes and at what rate, what it consumes, and how to start it:

```
healer startup_grace_s=3 verify_s=5

node camera lifecycle=1 fallback=camera_backup
out camera /image 30
exec camera @helix_demo/demo_node --ros-args -p lifecycle:=true -p rate_hz:=30.0 -p outputs:=[/image]

node camera_backup autostart=0
out camera_backup /image 15
exec camera_backup @helix_demo/demo_node --ros-args -p rate_hz:=15.0 -p outputs:=[/image]

node detector
in detector /image
out detector /detections 15
exec detector @helix_demo/demo_node --ros-args -p rate_hz:=15.0 -p inputs:=[/image] -p outputs:=[/detections]
```

`@pkg/exe` resolves through the ament index. Any other `exec` line is run as written.
`policy:=none|respawn|watchdog|helix` selects the baselines.

## Reproduce the evaluation

Each policy and scenario runs in its own ROS domain on a 4-core slice, with localhost-only Cyclone DDS.

```bash
python3 eval/heal/run_episodes.py --set manip --reps 3 --out eval/heal/out/heldout_manip   # about 21 min
python3 eval/heal/score.py eval/heal/out/heldout_manip --csv eval/heal/out/heldout_manip.csv
python3 eval/heal/learned.py --train eval/heal/out/dev_nav --test eval/heal/out/heldout_manip [--llm qwen3:8b]
```

`--set nav` reproduces the development numbers in [docs/v3/results/dev_nav.md](docs/v3/results/dev_nav.md). The
`--llm` arm needs a local Ollama server.

## Layout

```
src/helix_healer/   the healer: spec parser, process control, diagnosis/repair policy, node; gtests; nav and manip specs
src/helix_demo/     fault-injectable pipeline stage used by the evaluation
src/helix_monitor/  v2 GO2 sensor-integrity monitor and stop verifier (unchanged, see History)
src/helix_msgs/     messages used by helix_monitor
eval/heal/          episode runner, independent probe, scorer, learned-diagnosis arms
docs/v3/            method and results; docs/preregistration/ has the registrations
```

## History

- **v1** was a larger fault-detection and self-healing stack: anomaly scoring, diagnosis, an LLM fault explainer, a
  recovery node and a dashboard. Its recovery could not be verified. It is at tag `v0.3.0-wip-self-healing`, commit
  `86e4e60`.
- **v2** cut HELIX down to an observe-only monitor for the Unitree GO2. It had two pre-registered claims:
  - **Killed:** content-fault detection, by its false-alarm criterion (4.5 alarms/h against a limit of 2/h).
  - **Supported:** independent stop verification. Genuine stops were VERIFIED 36/36, and stops faked by the state
    estimator were refused 35/36.

  The monitor is still in the tree as `helix_monitor`, with its results in
  [docs/results/2026-11-02-helix-v2.md](docs/results/2026-11-02-helix-v2.md). It needs Cyclone DDS on the GO2,
  because Fast DDS silently drops the robot's oversized `SportModeState` samples. The v2-only tree is commit
  `0224520` on `arch/helix-v2`.
- **v3** (this README) brings back recovery as the goal and makes it measurable. Repairs are bounded and verified by
  an independent probe, and they are compared against conventional baselines on a pipeline the healer was not tuned
  on.

## Author

**Yusuf Guenena**
M.S. Robotics Engineering, Wayne State University
[linkedin.com/in/yusuf-guenena](https://linkedin.com/in/yusuf-guenena) &middot; [github.com/yusufdxb](https://github.com/yusufdxb)

## License

MIT. See [LICENSE](LICENSE).
