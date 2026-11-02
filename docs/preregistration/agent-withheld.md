# Pre-registration: HELIX repair agent on withheld fault families

Written before any agent code ran on any fault. The commit that adds this file is the
registration. A second commit, the **freeze**, comes after the developmental work in
step 1 below and before any withheld episode. It fixes the agent code, the trained MLP
weights, the LLM model name and prompt, and the history warm-start file. Nothing listed
here changes after the freeze.

## What the agent is

The C++ healer keeps detection, diagnosis and verification exactly as in v3 (`helix`
policy). The new `agent_*` policies add an incident layer on top:

- **Structured evidence**, collected deterministically by the healer. It covers:
  - the diagnosed fault, liveness, quick deaths and lifecycle state;
  - the parameter diff against the last-known-good (LKG) parameter snapshot;
  - the hash diff of the node's config files against their LKG copies, and whether the
    running process was started with the changed files;
  - the ERROR/FATAL count in the node's log tail;
  - the health of upstream nodes.
- **Closed actions**: Restart, LifecycleReset, ParamRestore (set only the parameters that
  differ from LKG), RestartLKG (restart with the LKG copies of the config files; the
  user's files are never written), Fallback, Report. No shell, no free-form commands.
- **Bounded retries**: at most K = 3 repair actions per incident. After that, the v3
  give-up path (fallback or SAFE_STOP) runs.
- **Rollback**: a ParamRestore whose repair fails verification is undone.
- **Independent verification**: v3's verifier. The node must be alive, show no symptom,
  and every declared output must publish fresh messages after the action.
- **Repair history**: an append-only outcome log keyed by evidence signature, readable
  by the rankers.

LKG parameters are snapshotted while the node is healthy. A snapshot is promoted only
after the node stays healthy for 5 s past it. LKG config files are copied at spawn and
promoted after 5 s of health.

The arms differ only in the **ranker** that orders the feasible, untried actions.

## Stack and fault families

The stack is the stock-Humble humanoid stack from `eval/heal/humanoid/make_stack.py`:
- controller_manager with mock hardware, /joint_states at 250 Hz;
- robot_state_publisher (rsp), /tf at 83 Hz;
- cam2image at 30 Hz, with a backup at 15 Hz.

No hardware is actuated.

**Development families** (`scenarios.py:HUMANOID`, already used in v3) are crash,
hang, stall, CPU starvation, controller deactivation, crash loop and flaky crash. Only
these are used to build and tune the agent, generate exploration data, train the MLP,
pick the LLM model and write the prompt.

**Withheld families** (`scenarios.py:AGENT_WITHHELD`) never run before the freeze:

| Scenario | Fault string | Root | Correct repair class |
|---|---|---|---|
| nominal | none | none | no action |
| w1_param_degrade | `param:rsp:publish_frequency:5.0` | rsp | param_restore |
| w1_param_stale | `param:rsp:publish_frequency:0.5` | rsp | param_restore |
| w2_rsp_bad_urdf | `deploy:rsp:rsp.yaml:corrupt_urdf` | rsp | config_rollback |
| w2_cm_rate | `deploy:controller_manager:controllers.yaml:set:update_rate:20` | controller_manager | config_rollback |
| w2_cm_bad_urdf | `deploy:controller_manager:controllers.yaml:corrupt_urdf` | controller_manager | config_rollback |
| safe_deploy_rsp | `deploy:rsp:rsp.yaml:set:publish_frequency:120.0` | rsp | restart (reverting is unsafe) |

What the injectors do:
- **W1**, runtime parameter corruption. The stock `ros2 param set` CLI changes the
  parameter on the live node.
- **W2**, bad config deploy. The node's config file in the episode stack is rewritten
  atomically, then the node gets SIGTERM, as an operator restart would send. The
  injector `deploy:` does this.
- **safe_deploy_rsp**, a legitimate deploy. The new config is valid, so after the
  restart the right outcome is to keep it.

### Premises checked before registration (injector targets only, no healer running)

| Check | Result |
|---|---|
| rsp with a truncated robot_description | aborts (rc 134, "Unable to initialize urdf::model") |
| controller_manager with a truncated robot_description | aborts (rc 134, "invalid URDF") |
| controller_manager with update_rate 20 | /joint_states 20.2 Hz |
| rsp with publish_frequency 120 | /tf 98.5 Hz, healthy |
| rsp publish_frequency set to 5.0 at runtime | /tf 83.5 Hz to 2.0 Hz; setting 100.0 restored it to 83.8 Hz (earlier this session) |

### Disclosure (A9)

The families were withheld from data, tuning, model selection and prompt writing. They
were **not** withheld from the designer: ParamRestore and RestartLKG were added knowing
that parameter and config faults exist.

Two things therefore follow:
- The withheld set tests whether the agent's evidence and rankers find the right repair
  without having seen these faults. It does not test whether a designer would think of
  these tools.
- A watchdog or v3 restart cannot repair a bad deploy by construction. The W2 gap
  between agent and helix measures a capability, not a smarter choice between the same
  tools. The ranker comparison (all agent arms share the same tools) is the part that
  measures choice.

An exploratory W3 may follow, but it is never confirmatory. After the freeze, a fresh
agent with no access to this design composes new scenarios from the injector primitives.

## Arms

| Arm | Purpose |
|---|---|
| agent_rule | Treatment and primary arm. Hand-written ranker: parameter diff first, then config rollback when the running process used the changed files, then restart, lifecycle reset, give-up. |
| helix | Named comparator: v3, same detection and verification, no incident layer, no new actions |
| watchdog | Conventional watchdog (restart on exit or stale output, fixed backoff, budget, fallback) |
| none | No-treatment control: the faults must actually break the stack |
| agent_random | Sham, A7 dose-matched: same evidence, candidates, budget K and verifier as every agent arm, order uniformly random (seeded per scenario and rep) |
| agent_history | Beta posterior per (evidence signature, action), optimistic for untried actions, warm-started from developmental exploration data, learns online across episodes |
| agent_mlp | Small MLP over (evidence features, action one-hot), trained on GPU on developmental exploration data only, C++ forward pass in the healer |
| agent_llm | Small local LLM (chosen on developmental data from qwen2.5:1.5b, qwen2.5:3b, llama3.2:3b and qwen3:4b-instruct, all small enough for a 16 GB Jetson Orin NX) returns a ranking constrained to the candidate enum by a JSON schema. On timeout or invalid output it falls back to the rule ranker; fallbacks are counted. |
| agent_oracle | Positive control and ceiling: ranks the scenario's correct repair class first, then the rule order |

## Claim (primary)

On the five withheld fault scenarios, agent_rule restores at least 0.3 more episodes per
(scenario, rep) block than helix. It does so with no more unsafe interventions than
helix on nominal and safe_deploy_rsp.

## Secondary hypotheses (exploratory unless stated)

- **S1.** agent_rule versus helix: shorter longest root-output blackout on the W1
  scenarios, where both should restore. This is a paired block-bootstrap CI, reported
  confirmatory under Holm with S2.
- **S2.** agent_rule versus agent_random: fewer actions to a verified repair on the
  withheld fault blocks. Exact sign-flip test, Holm with S1.
- **S3.** agent_history, agent_mlp and agent_llm versus agent_rule: restored rate and
  actions to repair. Exact sign-flip test, Holm across the three.

## Metrics (per episode, from committed artifacts)

**Outcome measures**
- **restored**: the independent probe's v3 definition. Every declared output is at 0.8x
  nominal over the last 3 s.
- **correct diagnosis**: the root decision's fault label is in the scenario's expect set.
- **correct repair**: the action that verified belongs to the scenario's repair class.
- **actions to repair**: the number of agent actions on the root until "verified" (K + 1
  when the incident was never verified).

**Timing and safety**
- **blackout**: the longest gap in the root's outputs after injection
  (`paired_blackout.py`).
- **downtime**: time from injection to the first moment from which the outputs stay OK.
- **unsafe intervention**, any of:
  - an action on a node that is neither root, collateral nor its fallback;
  - any action in nominal;
  - a RestartLKG or ParamRestore in safe_deploy_rsp (it reverts a legitimate change).

**Cost**
- **latency**: detect to act, and ranker time (logged by the healer).
- **resources**: healer CPU fraction and RSS (existing runner); advisor process CPU and
  RSS; GPU memory used by the LLM server during the arm.
- Jetson Orin NX latency is measured only if LabLink reports the device reachable.
  Otherwise it is reported as not measured, with no estimate.

## Run plan

1. **Developmental work** (labelled developmental, never confirmatory). Build and
   gtest. Smoke-test the new injectors with no healer. Run agent_random and agent_rule
   on `--set humanoid` for exploration data and regressions. Train the MLP. Pick the LLM
   model by valid-output rate and rule agreement on dev evidence. Commit the **freeze**.
2. **Withheld run**, one run, no reruns. Use `--set agent_withheld`, all 9 arms, 6 reps,
   5 slots, warm 12 s (so the LKG snapshots exist before injection), post 25 s. That is
   7 x 9 x 6 = 378 episodes. Injection times and random seeds are fixed per
   (scenario, rep). Every arm sees the same schedule.
3. Score with `score.py` plus the agent scorer.

If an episode crashes the harness, it is recorded as FAILED for that arm and not rerun.

## Analysis

**Units and tests**
- Unit: the (scenario, rep) block, paired across arms. Primary n = 30 fault blocks.
- Estimator: the mean paired difference in restored (0/1) with a 95% block bootstrap CI
  (10 000 resamples) and an exact sign-flip p value.
- Safety: the count of unsafe interventions per arm over the 12 nominal and
  safe_deploy_rsp episodes.
- Dose (A7): actions per episode and engagement rate, printed per arm and per scenario.
  The agent arms share K and the candidate set, so differences in dose come only from
  ordering.

**Guards on interpretation**
- **Positive control**: agent_oracle restores at least 90% of withheld fault episodes.
  If not, the pipeline is broken, nothing is interpreted, and that is the report.
- **Fault reality**: none restores at most 20% of withheld fault episodes. Otherwise the
  scenario does not break the stack and drops from the primary analysis, with a note.

## Kill criteria

- **Primary**: if the 95% CI of agent_rule minus helix (restored, 30 blocks) includes 0
  or its mean is below 0.3, the primary claim is dead and is reported as such. If
  agent_rule makes more unsafe interventions than helix on nominal and safe_deploy_rsp,
  the claim is dead regardless of recovery.
- **Learned rankers**: a learned ranker replaces the rule ranker as the shipped default
  only if it beats agent_rule on restored or actions to repair (Holm p < 0.05) without
  more unsafe interventions. Otherwise the rule ranker ships, and the learned arms are
  reported as measured negatives or ties with their costs.

Negative and null results are published in `docs/v3/agent/results.md` with the same
detail as positive ones.

## Not tested

- **Deep RL**: a DQN-style recovery policy needs on the order of a thousand fault
  episodes (see `docs/v3/agent/research.md`). The bandit-style history ranker is the
  sample-efficient form tested here.
- **A learned temporal diagnosis DNN**: detection and diagnosis stay v3's deterministic
  code in every arm. v3's earlier learned-diagnosis comparison (GBDT and an LLM
  classifier against the rules) was negative. The dev set has 13 scenarios, too few
  for a temporal model to be anything but memorized. This is a gap, not a finding.
