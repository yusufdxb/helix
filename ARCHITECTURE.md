# HELIX Architecture

HELIX is a four-tier self-healing layer for ROS 2 robots. It monitors the live
graph, classifies anomalies into recovery hints, applies a strict safety
envelope before any actuation, and produces operator-readable explanations.

The four tiers run as independent ROS 2 lifecycle nodes connected through
three HELIX topics:

| Tier | Package | Subscribes | Publishes |
|---|---|---|---|
| Sense | `helix_core`, `helix_sensing_cpp`, `helix_adapter` | live ROS 2 graph, `/helix/metrics` | `/helix/faults` (FaultEvent) |
| Diagnose | `helix_diagnosis` | `/helix/faults` | `/helix/recovery_hints` (RecoveryHint) |
| Recover | `helix_recovery` | `/helix/recovery_hints` | `/helix/cmd_vel` (Twist), `/helix/recovery_actions` (audit) |
| Explain | `helix_explanation` | `/helix/faults` | operator-facing summaries (advisory, off the safety path) |

The overview diagram lives in the project README (`## Architecture`); this
document is the prose detail.

## Sense

Three lifecycle nodes publish structured `FaultEvent` messages to
`/helix/faults`:

- **`anomaly_detector`** (`helix_core`, with a C++ port in
  `helix_sensing_cpp`). Subscribes to `/helix/metrics`, keeps a rolling
  per-metric window, and evaluates each new sample's z-score against the
  window *before* appending it, so the new sample cannot contaminate the
  baseline it is judged against. A fault fires only when the z-score crosses
  the configured threshold on `consecutive_trigger` samples in a row. NaN
  samples (from stale upstream topics) are counted as violations.
- **`heartbeat_monitor`** (`helix_core`). Subscribes to expected heartbeat
  topics. A fault fires when a monitored topic misses `miss_threshold`
  consecutive heartbeats within the configured timeout.
- **`log_parser`** (`helix_core`). Matches `/rosout` (or any configured log
  topic) against a regex rule set, emitting a fault on each match.

`helix_adapter` bridges robot-specific telemetry into the HELIX shape so the
sensing nodes can run against robots that do not natively publish
`/helix/metrics`. Three lifecycle nodes ship today: `topic_rate_monitor`
(per-topic message rate as `rate_hz/<topic>`; NaN when stale),
`json_state_parser` (extracts numeric and boolean fields from JSON payloads),
and `pose_drift_monitor` (rolling displacement rate).

## Diagnose

`helix_diagnosis` is a lifecycle node with a small state machine
(`IDLE` / `STOP_AND_HOLD`) and a first-match-wins rule table:

- **R1.** A `rate_hz/utlidar*` ANOMALY at ERROR severity emits a
  STOP_AND_HOLD hint with confidence 0.9.
- **R2.** A RESUME hint fires from a timer once the anomaly stream has been
  quiet for `anomaly_clear_seconds`.
- **R3.** A LOG_PATTERN fault emits a STOP_AND_HOLD hint.
- **R4.** A heartbeat-loss CRASH fault emits a STOP_AND_HOLD hint.

The rules live in `helix_diagnosis.rules` as pure functions and are
unit-testable without ROS 2. `RecoveryHint` messages are published to
`/helix/recovery_hints`.

A `/helix/get_context` client is created at configure time for future
context-aware rules; the rules currently match on `FaultEvent` fields alone
and the client is not yet exercised.

## Recover

`helix_recovery` is the only node in HELIX that publishes actuation
commands. It consumes `RecoveryHint` messages, applies the `SafetyEnvelope`
(enable flag, per-fault-type cooldown, action allowlist
`{STOP_AND_HOLD, RESUME, LOG_ONLY}`), and emits an audit `RecoveryAction`
on every decision (`ACCEPTED`, `SUPPRESSED_DISABLED`,
`SUPPRESSED_ALLOWLIST`, `SUPPRESSED_COOLDOWN`).

When holding a STOP, the node publishes a literal zero `Twist` to
`/helix/cmd_vel` at 20 Hz. The intended downstream is `twist_mux`, which
arbitrates HELIX (priority 100) against an operator joystick
(`/teleop/cmd_vel`, priority 200, which always wins) and the autonomy stack
(`/nav/cmd_vel`, priority 50). RESUME is exempt from cooldown so a safety
stop can never suppress its own release.

`auto_activate_recovery` is `false` by default in the bringup; recovery is
intentionally opt-in.

## Explain

`helix_explanation` is an advisory tier. It wraps a `llama-server` sidecar
(Qwen2.5-1.5B-Instruct Q4_K_M on the Jetson) and generates a plain-language
summary for each `FaultEvent`. The LLM call runs inside a
`ThreadPoolExecutor` so it cannot back-pressure the ROS executor;
schema-constrained JSON decoding (`response_format: json_schema`) keeps the
output parseable; a deterministic template is always published *first* and
the LLM annotation arrives later, if at all. The Recover allowlist is the
hard safety gate; the Explain tier never publishes actuation.

`llm_enabled` ships `false` by default. The Jetson llama-server deployment
runbook is in [`docs/LLAMA_SERVER_JETSON_SETUP.md`](LLAMA_SERVER_JETSON_SETUP.md);
it has not yet been exercised in a hardware session.

## Lifecycle and Bringup

All HELIX nodes are ROS 2 managed (lifecycle) nodes. `helix_bringup`
provides:

- `helix_sensing.launch.py` auto-transitions the three `helix_core` nodes
  through `configure -> active`.
- `helix_adapter.launch.py` does the same for the three `helix_adapter`
  nodes.
- `helix_closedloop.launch.py` brings up the full Sense + Adapter +
  Diagnose + Recover stack, plus an optional `twist_mux` with the canonical
  config at `src/helix_bringup/config/twist_mux.yaml`.
- `fault_injector` publishes synthetic anomalies for local testing.

## Design Choices

### Evaluate-before-append on the Z-score window

The anomaly detector evaluates `(sample - window_mean) / window_std`
against the *existing* history before appending the new sample. A spike
cannot inflate the baseline it is being compared against.

### Pure-function rule and envelope cores

`helix_diagnosis.rules` and `helix_recovery.SafetyEnvelope` are deliberately
ROS-free and pure-function. Every safety-relevant branch is exercised by
unit tests without an `rclpy` spin (allowlist, cooldown, RESUME exemption,
disabled rejection). This is what makes the recovery tier auditable from
the test suite alone.

### Recovery is the only `cmd_vel` publisher

By design, only `helix_recovery` writes to `/helix/cmd_vel`. The intended
downstream is `twist_mux` arbitrating HELIX against teleop and the autonomy
stack, so a node crash here results in `twist_mux` timing the input out and
emitting zero rather than an indeterminate command. The fail-safe behaviour
is asserted in `test_twist_mux_model.py`; hardware verification against the
real `twist_mux` under total input dropout is still owed.

### RESUME is exempt from cooldown

`SafetyEnvelope.evaluate` skips the cooldown check when the action is
`RESUME`, and a RESUME never writes a cooldown bucket. The cooldown exists
only to damp STOP_AND_HOLD flapping; a state-clearing release must not be
suppressible by the stop it is clearing. Regression tests cover this.

## Hardware Validation Boundary

Eight lab sessions on a Unitree GO2 with a Jetson Orin NX (2026-04-03 to
2026-04-23) cover persistent Jetson deployment, hardware-validated detection
on live LiDAR rate anomalies, ground-truth fault injection at ~1.8 s
end-to-end, and the Session 8 closed-loop run (30 anomalies, 14 hints, 14
audited actions, 3,064 zero-twist commands in a 7m19s bag).

What is *not* validated: continuous field deployment, multi-day stability,
and physical actuation closure. `/helix/cmd_vel` had zero downstream
subscribers in Session 8; wiring it through `twist_mux` on the robot is the
next milestone.

Full evidence: [`docs/GO2_HARDWARE_EVIDENCE.md`](GO2_HARDWARE_EVIDENCE.md).
Honest limits: [`docs/LIMITATIONS.md`](LIMITATIONS.md).
