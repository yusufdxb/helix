# HELIX v3 agent: withheld-family results

Registration: `docs/preregistration/agent-withheld.md` (with the host-change amendment).
Freeze: commit `1a4813e`, artifacts in `eval/heal/agent_freeze/`.
Raw per-episode data and scorer output: `docs/v3/results/agent_withheld*.{csv,txt,json}`.

Run: 7 scenarios x 9 arms x 6 reps = 378 episodes, 378/378 exited rc=0, 0 stray processes after the run.
Build identity (`agent_withheld_build.json`): head `1a4813e`, clean tree, binary sha256 `e496f7d3...`, binary sources match head, no provenance problems, `--allow-dirty` not used.

## Verdict

- **Primary claim survives.** The rule ranker (`agent_rule`) restores 0.600 more fault episodes per block than v3 `helix`, 95% block bootstrap CI [+0.433, +0.767], exact sign-flip p = 3.8e-6 (one-sided), n = 30 blocks. Unsafe interventions on nominal + safe_deploy: 0 vs 0. Kill criterion not met.
- **No learned ranker replaces the rule ranker.** History retrieval and the LLM tie it on restoration and use fewer actions, but both revert a legitimate deploy in 6/6 safe_deploy episodes. The MLP is worse on restoration and on actions. By the registered rule (beat rule, Holm p < 0.05, no more unsafe interventions) none qualifies. The rule ranker ships.
- **Rule ordering is not shown to beat random ordering** on actions to repair (S2 null).

## Guards

| Guard | Result |
|---|---|
| Positive control: `agent_oracle` restores >= 0.90 of fault episodes | 30/30 = 1.00, PASS |
| Fault reality: `none` restores <= 0.20 per fault scenario | 0.00 on all 5 fault scenarios, ok |

## Per arm (42 episodes each, 36 fault episodes)

| Arm | restored | diag_ok | repair_ok | actions to repair | unsafe | act/ep | engaged | downtime s | blackout s | act latency s | ranker ms | healer CPU | healer RSS MB |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| agent_rule | 42/42 | 33/36 | 36/36 | 1.50 | 0 | 1.57 | 36/42 | 2.83 | 0.57 | 0.76 | 0.003 | 0.015 | 18.1 |
| agent_oracle | 42/42 | 25/36 | 36/36 | 1.00 | 0 | 0.86 | 36/42 | 1.31 | 0.25 | 0.77 | 0.005 | 0.015 | 18.2 |
| agent_history | 42/42 | 28/36 | 18/36 | 1.11 | 6 | 0.98 | 36/42 | 1.50 | 0.38 | 0.62 | 0.009 | 0.015 | 18.1 |
| agent_llm | 42/42 | 26/36 | 18/36 | 1.00 | 6 | 0.86 | 36/42 | 1.73 | 0.42 | 1.12 | 237 | 0.015 | 18.1 |
| agent_random | 42/42 | 31/36 | 26/36 | 1.42 | 2 | 1.38 | 36/42 | 2.97 | 0.65 | 1.35 | 0.004 | 0.015 | 18.2 |
| agent_mlp | 36/42 | 34/36 | 18/36 | 2.17 | 0 | 2.00 | 36/42 | 5.89 | 1.16 | 0.55 | 0.009 | 0.015 | 18.2 |
| helix | 24/42 | 33/36 | 6/36 | 2.50 | 0 | 1.86 | 36/42 | 11.44 | 7.34 | 0.63 | - | 0.013 | 17.4 |
| watchdog | 18/42 | 30/36 | 6/36 | 3.00 | 0 | 2.21 | 30/42 | 14.55 | 7.38 | 0.28 | - | 0.013 | 17.3 |
| none | 6/42 | 0/36 | 0/36 | - | 0 | 0.00 | 0/42 | 21.43 | 14.60 | - | - | 0.014 | 17.3 |

Healer CPU is cores (fraction of one core). "actions to repair" counts an episode that never repairs as 4.

Dose (A7): every agent arm and `helix` engaged in exactly the 36 fault episodes and in 0 nominal episodes; `watchdog` engaged in 30/42 because it never sees the W1 degrade fault. The comparison among agent arms is therefore not a dose difference. Within that equal engagement, the arms differ in which action they pick, which is what is being tested.

## Per scenario: restored (of 6)

| Scenario | none | watchdog | helix | rule | random | history | mlp | llm | oracle |
|---|---|---|---|---|---|---|---|---|---|
| nominal | 6 | 6 | 6 | 6 | 6 | 6 | 6 | 6 | 6 |
| safe_deploy_rsp | 0 | 6 | 6 | 6 | 6 | 6 | 6 | 6 | 6 |
| w1_param_degrade | 0 | 0 | 6 | 6 | 6 | 6 | 6 | 6 | 6 |
| w1_param_stale | 0 | 6 | 6 | 6 | 6 | 6 | 6 | 6 | 6 |
| w2_cm_bad_urdf | 0 | 0 | 0 | 6 | 6 | 6 | 6 | 6 | 6 |
| w2_cm_rate | 0 | 0 | 0 | 6 | 6 | 6 | **0** | 6 | 6 |
| w2_rsp_bad_urdf | 0 | 0 | 0 | 6 | 6 | 6 | 6 | 6 | 6 |

All of `helix`'s deficit is in family W2 (bad config deploys): v3 restarts the node into the same bad config until the crash-loop fallback. The agent's `restart_lkg` (relaunch with the last-known-good config copy) is the action that repairs W2, and every agent arm except the MLP on `w2_cm_rate` reaches it.

## Per scenario: what each ranker did

- **safe_deploy_rsp** (a legitimate deploy followed by an unrelated crash; the repair is a plain restart and reverting is unsafe): rule, MLP and oracle restart (6/6 repair_ok). History and LLM revert to the LKG config in 6/6 episodes (unsafe). Random reverts in 2/6.
- **W1 param faults** (repair: `param_restore`): rule and oracle restore parameters 6/6. History, LLM, MLP and v3 `helix` restart instead; the restart also restores the node, so restoration ties, but it is not the repair class and costs a process relaunch.
- **W2 config faults** (repair: `restart_lkg`): LLM and oracle go to `restart_lkg` directly (1.00 actions). Rule restarts first and then reverts (2.00 actions, and higher W2 downtime: 5.44 s vs 1.44 s for the LLM on `w2_cm_bad_urdf`, 6.92 s vs 1.47 s on `w2_cm_rate`). The MLP restarts three times before reverting on the URDF faults and never reverts on `w2_cm_rate` (0/6).

The LLM and history arms win W2 and lose safe_deploy for the same reason: at the first decision the evidence in both families is "config changed on disk, process died", and they map that to a revert. The rule ranker only reverts first when the dying process was itself started from the changed files (`config_ran`). In this injector a deploy rewrites the file and terminates the old process, which was started from the LKG config, so `config_ran` is false at the first decision and the rule ranker restarts first. If the restart crashes on the new config, `config_ran` becomes true and it reverts. That is the designed behaviour, not a scoring artifact: it buys 0 unsafe reverts on safe deploys for one extra action on bad ones. It is also why S2 below is a null.

## Secondary analyses (Holm-corrected as registered)

| Test | Estimate | 95% CI | Holm p | Reading |
|---|---|---|---|---|
| S1 blackout on W1, rule - helix | -0.038 s | [-0.041, -0.036] | 0.0005 | significant but 38 ms; not practically meaningful |
| S2 actions to repair, rule - random | +0.100 | [-0.100, +0.333] | 0.887 | **null**: rule ordering not shown better than random ordering |
| S3 restored, history - rule | +0.000 | [0, 0] | 1 | tie |
| S3 restored, LLM - rule | +0.000 | [0, 0] | 1 | tie |
| S3 restored, MLP - rule | -0.200 | [-0.333, -0.067] | 1 (one-sided, wrong direction) | MLP worse |
| S3 actions, history - rule | -0.467 | [-0.667, -0.267] | 0.0005 | fewer actions, but 6 unsafe vs 0 |
| S3 actions, LLM - rule | -0.600 | [-0.767, -0.433] | 1.1e-5 | fewer actions, but 6 unsafe vs 0 |
| S3 actions, MLP - rule | +0.800 | [+0.533, +1.067] | 1 (wrong direction) | MLP worse |

S2 needs care: random reaches the same repairs because the candidate set per decision is small (one to three actions) and every candidate is safe to try except the revert in safe_deploy. Random paid for that with 2 unsafe reverts, which the action count does not show.

## Cost

| Item | Value |
|---|---|
| Rule ranker decision time | 0.003 ms mean |
| LLM ranker decision time | 237 ms mean (qwen2.5:1.5b, private single-slot server) |
| LLM fallbacks to rule | 0 |
| LLM server | 1.08 s load, 1290 MB GPU memory, 15.43 s server CPU over the 42-episode arm (attributable: private server) |
| Healer CPU, any arm | about 0.013-0.015 of one core |
| Healer RSS | 17.3-17.4 MB (v3 helix, watchdog) vs 18.0-18.2 MB (agent arms) |
| Median action latency, rule | 0.76 s (dominated by the detection window, not the ranker) |

## Diagnosis numbers are not a classifier comparison

All arms share the same deterministic fault classifier. `diag_ok` scores the fault label of the **last** root decision, so it moves with the action path: an arm that repairs on the first action is scored on its first diagnosis, an arm that needs a second action is scored on the post-restart diagnosis. Read `diag_ok` differences across arms as a path effect, not as diagnosis quality.

## Deviations from the registration

1. **Host change** (registered as an amendment before the run): the LLM runs on a private single-slot server on the workstation GPU, not on the laptop. The screen was rerun there and the laptop screens are excluded.
2. **Rebuild before the run.** The screen binary was built from sources that matched the freeze except for one healer log-format change already present at screening; provenance refused it as dirty. The healer was rebuilt from the clean freeze commit (tests 67/67) and the withheld run used that binary. The screened and run binaries differ in that build stamp and dirty flag only.
3. **LLM calls are serialized** through one server slot shared by the 5 parallel episode slots. The 237 ms mean includes queueing; it is an upper bound for a dedicated server.
4. **The rule ranker's W2 ordering** (restart, then revert) follows from `config_ran` as implemented and as described in the arm table; it is noted here because a reader of the arm description might expect a revert first on any config change. No code was changed after the freeze.

## Not measured

- **Orin latency and memory.** The deployment target was offline (LabLink gray) for the whole session. All latency and memory numbers above are workstation numbers.
- Deep RL and a temporal diagnosis network were not tested (registered as out of scope).
- No hardware actuation: every episode runs ROS 2 software stacks only.

## What ships

The rule ranker over the closed action set, with bounded retries, the LKG rollback, and independent verification. History retrieval and the LLM ranker stay available as opt-in arms; the evidence says they need a deploy-legitimacy signal before they could be trusted with revert decisions, and that is the open problem this run exposed.
