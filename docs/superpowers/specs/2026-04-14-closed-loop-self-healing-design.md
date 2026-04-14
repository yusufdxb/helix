# HELIX Closed-Loop Self-Healing — Design Spec

**Date:** 2026-04-14
**Author:** Yusuf Guenena
**Status:** Approved (brainstorming)
**Scope:** Diagnosis tier + recovery tier, as one coherent closed-loop spec

---

## 1. Goal

Turn HELIX from a sensing-only prototype into a closed-loop self-healing system suitable for an industry-facing demo. The system must detect a sensor fault on a live Unitree GO2, diagnose it, issue a safe recovery action that visibly changes the robot's behavior, and resume normal operation when the fault clears. The entire pipeline must be filmable in under two minutes and defensible to a senior roboticist's review.

**Not in scope for this spec:** completion of the sensing tier (tracked separately in `hardware_run_prompt3` on T7), the LLM inference upgrade (v2, one-file swap from the stub shipped here), recovery execution beyond stop/resume (future work), dashboard / persistent fault storage (future work).

## 2. Design decisions (approved)

| # | Decision | Choice |
|---|---|---|
| 1 | Spec scope | Diagnosis + Recovery as one coherent closed loop |
| 2 | Demo scenario | Sensor-fault-triggered behavioral recovery (LiDAR occlusion → stop → resume) |
| 3 | Control interface | `twist_mux` priority arbitration |
| 4 | Diagnosis approach | Deterministic rules (v1); LLM as parallel explainer sidecar (v1 stub, v2 real) |
| 5 | Development strategy | Sim-first using the existing Isaac Sim GO2 bridge; hardware for final validation |
| 6 | Safety envelope | Minimum viable: enable flag + per-fault cooldown + action allowlist + `twist_mux` fail-safe timeout |

## 3. Architecture

```
[GO2 sensors / topics]
      ↓
┌───────────────────────────────────────────────────────┐
│ Tier 1 — Sensing  (EXISTS)                            │
│   AnomalyDetector, HeartbeatMonitor, LogParser        │
│   PassiveAdapter: GO2 topics → /helix/metrics         │
│   output: /helix/faults  (FaultEvent)                 │
└───────────────────────────────────────────────────────┘
      ↓
┌───────────────────────────────────────────────────────┐
│ Tier 2 — Context  (NEW, small)                        │
│   ContextBuffer: /rosout ring (last 200 msgs),        │
│                  latest /helix/metrics snapshot,      │
│                  latest /helix/node_health.           │
│   no output topic; queried by Tier 3 via service      │
└───────────────────────────────────────────────────────┘
      ↓
┌───────────────────────────────────────────────────────┐
│ Tier 3 — Diagnosis  (NEW)                             │
│   DiagnosisNode: FaultEvent + ContextBuffer query →   │
│     deterministic rule evaluator                      │
│   output: /helix/recovery_hints  (RecoveryHint)       │
└───────────────────────────────────────────────────────┘
      ↓
┌───────────────────────────────────────────────────────┐
│ Tier 4 — Recovery  (NEW)                              │
│   RecoveryNode: consumes RecoveryHint,                │
│     enforces safety envelope, publishes Twist         │
│   output: /helix/cmd_vel  (priority input to twist_mux)│
│           /helix/recovery_actions  (audit)            │
└───────────────────────────────────────────────────────┘
      ↓
   twist_mux  (priority arbitration, fail-safe timeout 500ms)
      ↓
   [GO2 /cmd_vel]


Parallel sidecar:
┌───────────────────────────────────────────────────────┐
│ LLMExplainer  (NEW, v1 stub / v2 Ollama)              │
│   joins FaultEvent + RecoveryHint by fault_id         │
│   output: /helix/explanations  (String)               │
│   NOT in critical path. No LLM in v1.                 │
└───────────────────────────────────────────────────────┘
```

### Key architectural invariants

- **Only `RecoveryNode` publishes actuation commands.** Every other node is read-only w.r.t. robot state.
- **The safety envelope lives in `RecoveryNode` alone.** Diagnosis can emit any hint it likes; Recovery is the single chokepoint that enforces enable-flag, cooldown, and allowlist.
- **`twist_mux` is the fail-safe.** If `RecoveryNode` crashes, `twist_mux` times out its input within 500 ms and nav/teleop resume.
- **LLM is architecturally parallel, not in series.** Its presence or absence does not change the action the robot takes.

## 4. Components

### 4.1 Existing components (referenced, not changed in this spec)

| Component | Role in this design |
|---|---|
| `AnomalyDetector` | Source of `ANOMALY`-class `FaultEvent` on rate-based metrics (demo-critical). |
| `HeartbeatMonitor` | Source of `CRASH`-class `FaultEvent` (feeds rule R4). |
| `LogParser` | Source of `LOG_PATTERN`-class `FaultEvent` (feeds rule R3). |
| `PassiveAdapter` | Bridges GO2 topic rates to `/helix/metrics` so `AnomalyDetector` can observe them. |

### 4.2 New components

#### `ContextBuffer`

- **Purpose:** maintain a small, bounded history of `/rosout`, recent `/helix/metrics` values, and `/helix/node_health` snapshots, so diagnosis has context beyond a single `FaultEvent`.
- **Package:** `helix_diagnosis` (new ROS 2 package).
- **Node type:** lifecycle node.
- **Subscriptions:** `/rosout` (keep ring of 200), `/helix/metrics` (keep latest), `/helix/node_health` (`DiagnosticArray`, keep latest).
- **Services:** `GetContext` — returns a `ContextSnapshot` message containing the current ring + latest snapshots. No parameters; the node keeps one active snapshot.
- **State:** in-memory only. No persistence.
- **Constraints:** single-file, pure Python, no external deps beyond `rclpy`.

#### `DiagnosisNode`

- **Purpose:** turn a `FaultEvent` + context into a `RecoveryHint` via a pure, deterministic rule function.
- **Package:** `helix_diagnosis`.
- **Node type:** lifecycle node.
- **Subscriptions:** `/helix/faults` (`FaultEvent`).
- **Clients:** `GetContext` on `ContextBuffer`.
- **Publications:** `/helix/recovery_hints` (`RecoveryHint`).
- **Rule evaluator lives in `helix_diagnosis/rules.py`** as a pure function: `evaluate(fault_event, context) -> Optional[RecoveryHint]`. This file is independently unit-testable without ROS 2.

#### `RecoveryNode`

- **Purpose:** single chokepoint that translates `RecoveryHint`s into robot commands under safety constraints.
- **Package:** `helix_recovery` (new ROS 2 package).
- **Node type:** lifecycle node.
- **Subscriptions:** `/helix/recovery_hints` (`RecoveryHint`).
- **Publications:**
  - `/helix/cmd_vel` (`geometry_msgs/Twist`) — priority input to `twist_mux`.
  - `/helix/recovery_actions` (`RecoveryAction`, new msg) — audit topic.
- **Parameters:**
  - `enabled` (bool, default `false`) — master enable flag.
  - `cooldown_seconds` (float, default `5.0`) — per-fault-type cooldown.
  - `allowed_actions` (string[], default `['STOP_AND_HOLD', 'RESUME', 'LOG_ONLY']`).
- **State:** in-memory `dict[str, Time]` tracking last action time per fault type; in-memory `str` tracking current recovery state (`IDLE`, `STOP_AND_HOLD`).
- **Publish cadence in `STOP_AND_HOLD`:** zero-velocity `Twist` at 20 Hz so `twist_mux` keeps the input fresh (within the 500 ms timeout).

#### `LLMExplainer` (v1 stub)

- **Purpose:** emit a human-readable explanation of each fault + recovery decision, out of the critical path.
- **Package:** `helix_explanation` (new ROS 2 package).
- **Node type:** standard node (not lifecycle).
- **Subscriptions:** `/helix/faults`, `/helix/recovery_hints`.
- **Publications:** `/helix/explanations` (`std_msgs/String`).
- **v1 implementation:** template-based string formatting. No LLM, no external deps.
- **v2 upgrade path:** replace the formatting call with an Ollama inference call. Single-file change. Timeout + fallback-to-template on LLM failure. Not in this spec.

#### `FaultInjector` (extended)

- **Status:** exists in `src/helix_bringup/helix_bringup/fault_injector.py` as a demo tool. Extend it with sim-targeted modes.
- **New modes:**
  - `lidar_rate_drop` — publishes synthetic low-rate `/utlidar/cloud` messages or drops a configured fraction.
  - `node_crash` — kills a named node via `os.kill` on its PID.
- **Used by the sim harness** (section 5), not in the demo itself.

### 4.3 Messages

#### Extended `RecoveryHint.msg` (existing, add one field)

```
string   fault_id
string   suggested_action        # STOP_AND_HOLD | RESUME | LOG_ONLY
float32  confidence              # 0.0 - 1.0
string   reasoning               # short, human-readable
string   rule_matched            # NEW — rule ID for audit
```

#### New `RecoveryAction.msg`

```
string   fault_id
string   action                  # the action that fired, or was suppressed
string   status                  # ACCEPTED | SUPPRESSED_COOLDOWN | SUPPRESSED_ALLOWLIST | SUPPRESSED_DISABLED
float64  timestamp
string   reason                  # human-readable for the audit log
```

#### New `ContextSnapshot.srv`

Request: empty. Response:
```
string[]  rosout_ring             # last 200 lines, oldest first
string    metrics_json            # latest /helix/metrics as JSON string
string    node_health_json        # latest /helix/node_health as JSON string
float64   snapshot_time
```

## 5. Data flow — demo scenario end-to-end

1. GO2 is walking. `/utlidar/cloud` publishes at ~10 Hz. `PassiveAdapter` republishes the rate on `/helix/metrics`.
2. Operator physically occludes the LiDAR. Within ~500 ms, the `/utlidar/cloud` rate measured on `/helix/metrics` drops below the anomaly threshold.
3. `AnomalyDetector` fires a `FaultEvent(fault_type=ANOMALY, metric_name='utlidar_rate', severity=ERROR, context_keys=[...])` on `/helix/faults`.
4. `DiagnosisNode` receives the fault, calls `GetContext`, evaluates rules, emits `RecoveryHint(action=STOP_AND_HOLD, confidence=0.9, reasoning='lidar_rate_degraded', rule_matched='R1')` on `/helix/recovery_hints`.
5. `RecoveryNode` receives the hint. Checks: `enabled==true`? cooldown not active for `ANOMALY`? action in allowlist? All pass. Publishes `Twist(0,0,0)` on `/helix/cmd_vel` at 20 Hz and logs `ACCEPTED` to `/helix/recovery_actions`.
6. `twist_mux` sees HELIX's high-priority input and forwards its zero-velocity Twist to `/cmd_vel`. GO2 stops.
7. Operator removes the occlusion. Rate recovers. `AnomalyDetector` exits anomaly state after the configured clear window (typically 3 s).
8. `DiagnosisNode` emits `RecoveryHint(action=RESUME, rule_matched='R2')`. `RecoveryNode` stops publishing on `/helix/cmd_vel`.
9. `twist_mux` times out HELIX's input (~500 ms). Nav/teleop regains control. GO2 resumes.
10. `LLMExplainer` publishes a string per step 3 and per step 8 to `/helix/explanations` for the demo overlay.

Every decision visible in `/helix/recovery_actions` and every explanation in `/helix/explanations` is bagged for the demo artifact.

## 6. Recovery rules (v1)

All rules live in `helix_diagnosis/rules.py` as pure functions registered in an ordered list. First match wins.

| # | Trigger | Action | Purpose | Required? |
|---|---|---|---|---|
| R1 | `fault_type == 'ANOMALY'` AND `metric_name == 'utlidar_rate'` AND `severity >= ERROR` | `STOP_AND_HOLD` | demo-critical stop | yes |
| R2 | no active `ANOMALY` for >= 3 s AND current recovery state is `STOP_AND_HOLD` | `RESUME` | demo-critical resume | yes |
| R3 | `fault_type == 'LOG_PATTERN'` AND `severity == CRITICAL` | `STOP_AND_HOLD` | defensive, cheap to add | yes |
| R4 | `fault_type == 'CRASH'` | `LOG_ONLY` | non-actuating, signals scope boundary | yes |

R2 is stateful. The `DiagnosisNode` keeps a small state machine (`IDLE` / `STOP_AND_HOLD`) driven by recent `FaultEvent` history, so R2 only fires when R1 has previously fired.

## 7. Safety envelope (concrete)

| Mechanism | Implementation |
|---|---|
| **Enable flag** | `RecoveryNode` param `enabled` (bool, default `false`). Launch file for the demo flips it `true`. Param updates respected via on-set-parameters callback. When `false`: node still subscribes, still logs to audit, but never publishes Twist. |
| **Per-fault cooldown** | In-memory `dict[str, Time]` keyed by `fault_type`. Before firing, check `now - last_action[fault_type] >= cooldown_seconds`. If not, log `SUPPRESSED_COOLDOWN` and drop. |
| **Action allowlist** | Python set `ALLOWED_ACTIONS = {'STOP_AND_HOLD', 'RESUME', 'LOG_ONLY'}`. Hints outside the set logged as `SUPPRESSED_ALLOWLIST` and dropped. |
| **twist_mux timeout** | `config/twist_mux.yaml` sets HELIX's input topic timeout to 0.5 s. |
| **Audit log** | Every decision (accepted or suppressed) published to `/helix/recovery_actions` and bagged during demo runs. |

## 8. Simulation harness

Built on the existing, working `~/IsaacLab/scripts/reinforcement_learning/rsl_rl/go2_ros2_bridge.py` (verified 2026-04-14 end-to-end for `/go2/cmd_vel`, `/go2/odom`, `/go2/joint_states`, `/go2/imu`).

**Sim topic bridging:** the Isaac Sim bridge currently publishes `/go2/*` topics and does not publish `/utlidar/cloud` (the topic the real GO2 publishes and that `PassiveAdapter` monitors). Two options for P3:

1. **Preferred — extend the sim bridge** to publish a synthetic `/utlidar/cloud` (can be a fixed-rate empty `PointCloud2` — detector only cares about the rate, not the contents).
2. **Fallback** — reconfigure `PassiveAdapter` for the sim run to monitor `/go2/joint_states` rate instead. Still proves the closed loop, but the demo topic differs from the hardware demo topic, which weakens the "same test in sim and on hardware" claim.

Decision: attempt option 1 first; fall back to option 2 if it costs more than a day.

**New tooling under `scripts/sim_faults/`:**

| Script | Purpose |
|---|---|
| `inject_lidar_rate_drop.py` | Relay node that intercepts `/utlidar/cloud` (or an equivalent sim topic) and throttles it to a configurable Hz on a schedule. Default: 10 Hz × 30 s → 1 Hz × 20 s → 10 Hz × 30 s. |
| `inject_node_crash.py` | Kills a named HELIX node after N seconds to exercise R4 + `HeartbeatMonitor`. |
| `run_closed_loop_scenario.py` | Orchestrator. Starts the Isaac Sim bridge, all HELIX lifecycle nodes, `twist_mux`, an injector, and records a bag + screenshots. Exit code is the scenario result. |

**Integration test at `tests/sim_integration/test_lidar_occlusion_recovery.py`:**

Runs `run_closed_loop_scenario.py` with the LiDAR-drop injector and asserts:
1. `FaultEvent` on `/helix/faults` within 1 s of rate drop.
2. `RecoveryHint(action=STOP_AND_HOLD)` on `/helix/recovery_hints` within 100 ms of that.
3. `Twist(0,0,0)` on `/helix/cmd_vel` within 100 ms of the hint.
4. Sim robot linear velocity drops to < 0.05 m/s within 1 s.
5. After sim rate recovers, `RecoveryHint(action=RESUME)` within ~3 s.
6. Sim robot linear velocity returns to > 0.1 m/s.

This is the closed-loop claim in code. It is the single most load-bearing test in the repo.

## 9. Testing strategy

| Tier | Scope | Where it runs | When |
|---|---|---|---|
| Unit | `rules.evaluate`, safety envelope checks, cooldown timer, allowlist filter | CI (GitHub Actions), local `pytest` | every push |
| Sim integration | Full closed-loop scenarios | Local, `make test-sim` target | before each merge to `main` |
| Hardware validation | Demo scenario on live GO2 | One lab session | once, pre-demo-filming |
| Regression | Re-run sim integration after every change to rules / safety envelope | Local | before every merge |

CI cannot run Isaac Sim. The integration test is gated behind a local Make target, documented in README, with a CI-side dry check that the harness imports cleanly.

## 10. Deliverables

| Deliverable | Description |
|---|---|
| Feature branch | `feat/self-healing-closed-loop`. Merged to `main` phase by phase. |
| Updated `README.md` | New architecture section (4-tier pipeline), demo GIF, quick-start for local sim, honest limitations section. |
| Updated architecture SVG | Regenerated from Graphviz source reflecting the 4-tier pipeline + arbitration layer. |
| Demo video (~2 min) | Three shots: uncontrolled baseline, HELIX-disabled fault (robot keeps walking), HELIX-enabled fault (robot stops + resumes). Overlay from `/helix/explanations`. Narrated. |
| Blog post (~1500 words) | Topic: tiered safety-critical architecture with deterministic rules + LLM as explainer. Draft in `docs/blog/closed-loop-self-healing.md`; final hosted on Yusuf's site. |
| Session 3 artifact | On T7: `LABWORK/HELIX/hardware_eval_20260???/` following the Session 2 structure. Bags gitignored per existing pattern. |
| Closed-loop integration test | `tests/sim_integration/test_lidar_occlusion_recovery.py`. |

## 11. Phased execution plan (high-level — detailed plan follows in the writing-plans step)

| Phase | Scope | Gate to next phase |
|---|---|---|
| P1 | Scaffold packages + messages (`helix_diagnosis`, `helix_recovery`, `helix_explanation`), extend `RecoveryHint.msg`, add `RecoveryAction.msg` + `ContextSnapshot.srv`. | Build passes, `ros2 interface show` resolves all. |
| P2 | Implement `rules.py` (pure) + unit tests. Implement `ContextBuffer` + `DiagnosisNode` + `RecoveryNode` (safety envelope complete) + `LLMExplainer` stub. | All unit tests pass. Nodes launch and pass smoke test with `enabled=false`. |
| P3 | `twist_mux` integration. Sim fault-injection harness (`inject_lidar_rate_drop.py`, `run_closed_loop_scenario.py`). | Sim integration test passes end-to-end for LiDAR-occlusion scenario. |
| P4 | Closed-loop integration test committed. Regression coverage for R3, R4. `make test-sim` target wired. | All four rules exercised in sim. |
| P5 | Hardware validation session on live GO2 + Jetson. Capture bags, overlays, final demo footage. | All six `FaultEvent`-class asserts from section 8 pass on hardware. |
| P6 | Demo video editing, blog post drafting, README + SVG updates, Session 3 artifact to T7. | All deliverables shipped. Merge to `main`. |

## 12. Risks and mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| Sim LiDAR rate behavior doesn't match real LiDAR rate behavior | Medium | P5 explicitly compares. Any discrepancy is documented honestly (not hidden); rule thresholds may need tuning between sim and hardware. |
| `twist_mux` not actually deployable on GO2 in time for P5 | Low | `twist_mux` is already observed on GO2's `/diagnostics` source. If blocked, fall back to publishing directly on `/cmd_vel` with a clear documentation note about the trade-off. |
| `RecoveryNode` oscillates between `STOP_AND_HOLD` and `RESUME` at the R2 boundary | Medium | Cooldown + R2 requires 3 s clear window + the `IDLE` / `STOP_AND_HOLD` state machine. Sim harness explicitly tests borderline cases. |
| Hardware session reveals a fault class that the rules don't handle | Medium | This is expected. Document it. Failing gracefully (no action > wrong action) is the design intent. |
| LLM sidecar adds confusion about what fired vs what explained | Low | Audit log (`/helix/recovery_actions`) is the ground truth. `/helix/explanations` is explicitly narrative only. Distinction called out in README. |

## 13. Out of scope (explicit)

- Recovery execution beyond `STOP_AND_HOLD` / `RESUME` / `LOG_ONLY` (no behavioral maneuvers, no node restart, no degraded-mode switching — future work).
- Multi-fault concurrent recovery (one fault at a time, first-match-wins).
- Learning or adapting rules from data.
- Real LLM inference in the critical path (v2 swap is a narrative upgrade only).
- Dashboard, persistent fault storage, LLM fine-tuning.
- Sensing-tier gaps (`hardware_run_prompt3`) — handled separately.

## 14. Success criteria

1. `tests/sim_integration/test_lidar_occlusion_recovery.py` passes reliably (≥ 9/10 runs).
2. One hardware session produces the demo footage with the same six asserts holding on real GO2.
3. Demo video filmed, edited, under 2:30 minutes, with on-screen overlay from `/helix/explanations`.
4. Blog post drafted (~1500 words), covers architecture + safety + LLM-as-explainer reasoning.
5. Repo `README.md` updated with new architecture and a local `make test-sim` path that works on mewtwo.
6. Merged to `main` with clean history.
