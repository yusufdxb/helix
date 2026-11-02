# Pre-registration: HELIX v3 held-out evaluation (MANIP topology)

Date: 2026-11-02. Written before any MANIP fault episode was run, and
completed with the DEV (NAV, 3 reps) results before the freeze commit. The DEV set
(NAV topology, `eval/heal/scenarios.py:NAV`) was used for development and bug
fixing; MANIP (`src/helix_healer/config/manip.spec`, `scenarios.py:MANIP`) was
written before it ran and is used once, with no tuning afterwards.

Frozen code: the commit that adds this file. Healer thresholds, policy rules,
scenario list, scoring rules and learned-diagnosis features are not changed
after it. The single allowed MANIP run before the held-out run is one
nominal-only episode per policy, to check that the topology starts and that
every output reaches its nominal rate (an engineering check, no faults; its
output is reported).

## Study 1: repair policy

### Claim

On the held-out MANIP topology (14 fault scenarios x 3 reps), HELIX
(diagnose, targeted repair, verify, fall back) ends with every output restored
in at least 3 more episodes than the conventional watchdog (restart on crash,
stale output, or RSS limit, with the same retry budget, verification and
fallback), with no more unnecessary interventions.

Prediction from DEV (stated so it can be wrong): the gap comes from rate
degradation (watchdog cannot see a node that publishes slowly but is not stale)
and from fewer collateral restarts of starved downstream nodes.

### Arms

| Arm | Purpose |
|---|---|
| `helix` | treatment: diagnosis-specific action (restart, lifecycle reset, fallback, SAFE_STOP) with verification |
| `none` | no-treatment control: same healer process and monitoring, never acts. Every restorable fault must end FAILED here (positive control that faults really impair outputs) |
| `respawn` | the `respawn=true` baseline in ROS 2 launch: restart only when the process exits |
| `watchdog` | strong conventional baseline and dose comparator: acts on symptoms (dead, stale, RSS) with the same budget, verification window and fallback as HELIX. It differs from HELIX only in *which* node and *which* action |
| oracle (analytic) | every restorable scenario RESTORED with 0 unnecessary decisions; non-restorable ones (qos_mismatch, crash_loop_nobackup) SAFE. Bounds the table |

A7, dose: decisions per episode are printed per arm and per scenario
(`decisions`, `unnecessary`, `futile` columns of `score.py`). A HELIX win that
comes with more decisions than watchdog is reported as such.

Positive controls: `nominal` must be RESTORED with 0 decisions under every
policy; `crash_segv` must be RESTORED under `respawn`; every restorable fault
except `cpu_runaway` and `leak` must not be RESTORED under `none`. Those two
are resource faults: on DEV (3 reps) they ended RESTORED under `none` because a
runaway or leaking process starves nothing within the 25 s window on a 4-core
slice. Their harm is the resource still burning at the end (`burning` column,
3/3 under `none` on DEV), so for them `burning` is the outcome that must appear
under `none`. They stay in the RESTORED totals for every arm, where they tie.

### Confounds

- A1 onset: faults are injected 7 s after start (startup grace 3 s); every
  episode prints `pre_ok` (all outputs at nominal before injection). Episodes
  with `pre_ok=False` or `inject_subs=0` are reported and excluded from both
  arms of the pair.
- A2 exposure: all arms run the same 7 s + 25 s window; outcome is judged on
  the same last 3 s.
- A3 reps: 3 reps per (scenario, policy). The unit of pairing is the scenario
  (14 blocks). Primary test: exact one-sided sign-flip permutation on
  per-scenario differences in RESTORED count (helix minus watchdog),
  `score.py:primary_test`; floor with 14 non-tied blocks is 2^-14. With ties the
  floor rises; it is printed. On DEV only 2 of 12 blocks were non-tied
  (floor 0.25), so the held-out test is expected to be underpowered at the
  scenario level: the decision rule is the kill criterion below, and the
  p-value is reported as is. No significance is claimed unless p < 0.05.
- A4 bundling: both arms are the same binary, spec, budget (3 attempts),
  verify window (5 s), fallback map and probe; only `policy:=` differs.
- A6 outcome: RESTORED is measured by an independent probe process on the
  output topics (rate >= 0.8 x nominal over the whole last 3 s), not by the
  healer's own claims.
- A8 ceiling: on DEV (3 reps, 39 episodes per arm) watchdog restored 30/39
  and HELIX 36/39 (the other 3 are the non-restorable QoS mismatch, SAFE in
  both). Most blocks tie at the ceiling; the gap can only come from scenarios
  that a symptom-only supervisor cannot see.
- A9 selection: DEV results drove two healer fixes (lifecycle `get_state`
  retry, Hang mapped to restart) and one scorer fix in `learned.py` (a node on
  the tick it died read like a never-started backup and was dropped from the
  candidates, which hid every crash from the GBDT and the LLM; candidates are
  now nodes that were ever alive). MANIP was written before running; nothing is
  tuned on it.
- A10 leakage: none for study 1 (rule-based, no fit).

### Kill criterion

If HELIX's RESTORED total exceeds watchdog's by fewer than 3 episodes out of 42,
or HELIX has more unnecessary decisions than watchdog, the claim "HELIX restores
more than a strong watchdog" is dead for this topology and is published as such.
Recovery latency, outage time, futile repairs, left-burning resources and
healer CPU/RSS are reported as secondary, never as the headline.

## Study 2: learned diagnosis vs rules

### Claim

A gradient-boosted classifier trained on DEV (NAV) observations, or a local
LLM restricted by JSON schema to the closed node and fault enums, identifies
(root node, fault class) on held-out MANIP observations at least 2 episodes
more often than the deterministic HELIX rules, with no more false alarms.

### Arms

| Arm | Purpose |
|---|---|
| rules | HELIX's deterministic diagnosis (the decision it logged) |
| GBDT | sklearn HistGradientBoosting on 18 name-agnostic per-node features (`eval/heal/learned.py`), trained on DEV NAV only |
| LLM | Ollama, GPU, zero-shot, schema-constrained to node names + fault enum; the model chooses a label, never an action or a command. Models: qwen3:4b-instruct and qwen3:8b (qwen2.5:3b crashes the shared Ollama server's multi-user cache) |
| oracle | the scenario's ground truth label (ceiling = n/n) |
| positive control | the GBDT scored on DEV (train) episodes must be well above chance; if it is not, the pipeline is broken and a held-out null is uninterpretable. Result before freeze: metric A 36/36 (rules 36/36), false alarms 4/3160 fault-free snapshots (0.13%), detection 0.30 to 1.06 s |

### Confounds

- Selection bias: metric A uses the snapshot at which the rules decided, which
  favours the rules (they fired on it). Metric C removes this: on `none`
  episodes the GBDT's time to flag the right (root, class) for 2 consecutive
  snapshots is compared with HELIX's own first-root-decision latency
  (`act_latency_s`) on the same scenario.
- Leakage: train and test are different topologies with different node names;
  features use no names.
- False alarms: GBDT flags on fault-free snapshots (pre-injection, after the
  3 s grace) vs HELIX's unnecessary decisions in `nominal`.

### Kill criterion and integration rule

The learned diagnoser is integrated into the healer only if, on held-out MANIP,
its decision-point accuracy (metric A) is at least rules + 2 episodes AND its
false-alarm rate is no higher than the rules' AND its median detection time is
no slower than HELIX's median act latency. Otherwise the deterministic rules
stay and the learned result is published as a negative.
