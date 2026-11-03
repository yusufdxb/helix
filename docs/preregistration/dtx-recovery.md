# Pre-registration: deployment-aware transactional recovery (`dtx`)

The commit that adds this file is both the registration and the freeze. The healer binary
(sources at `c15656b`), the `dtx` scenario set (`8482a33`), the injector
(`eval/heal/humanoid/inject.py`) and the scorers (`eval/heal/score_dtx.py` at `0a39b99`, which
uses `score_gen.gen_row`, `score_agent.compare/holm` and `score_txn.table` unchanged) stay
fixed after it. The run uses exactly this HEAD; provenance refuses a dirty tree or a binary
built from another commit.

## Why this study, and what it is not

The gen study (`docs/v3/agent/gen-results.md`) showed that versioned snapshots can recover
failures that single-generation rollback cannot: a poisoned last-known-good (LKG) config was
repaired 4/4 by gen and 0/4 by txn2. But gen also reverted a valid deploy 4/4. The open
problem is the decision to revert, not the mechanism. Crash timing cannot answer it, and an
acknowledgement of who authorized a deploy cannot answer it either: an authorized operator
can deploy a bad config on purpose (the gen-ack-confirm study was withdrawn for this reason).

`dtx` answers it with evidence about the deployment's behavior. For every config hash a node
runs, the healer keeps a deployment history and samples the node's declared output rates once
per second. A sample is taken only when the process has been up longer than the window
W = 3 s and no managed upstream has been unhealthy in the window, so that an upstream outage
is not blamed on the config. A sample passes when every declared output's rate over W is at
least 0.85 x its spec rate. With at least 3 samples a config gets a verdict
(`policy.hpp:deploy_verdict`): +1 (verified) when good >= 3 and good >= bad, -1 (failed) when
bad >= 3 and bad > good, else 0. Decisions:

- verdict -1 and a failure: revert to the newest generation whose verdict is not -1, on the
  first failure (no second strike);
- verdict +1: never reverted, a crash only restarts it;
- verdict 0 (not enough evidence): behaves exactly as txn2 (two strikes, then revert an
  uncommitted config);
- a config with verdict -1 is never committed as a generation and never a rollback target.
  A generation is trusted only after probation (30 s) and verdict +1.

This is a short mechanism check against the two best prior arms, not a confirmatory
architecture study. Its limit is stated up front: dtx can only see a bad config whose fault
shows in the output rates before the delayed failure. A silent bad config (`c3_late_cfg_late`)
is in the run as a registered cost; dtx is predicted to fail it like txn2.

## Substrate changes since the gen study (disclosed; they change rule@5 and txn2 too)

The scenario smokes below exposed three defects in the healer's shared substrate, fixed in
`c15656b` with unit tests. They apply to every agent arm, so rule@5 and txn2@10 in this run
are not byte-identical to their gen-study versions:

1. **Deploy rollout restart uses no budget.** A crash while a deploy is pending (disk config
   hash differs from the running hash, and is not the pinned rejected config) gets a plain
   Restart with detail `deploy_rollout`: no incident, no repair budget, no candidate ranking.
   Before, the restart a deploy needs counted as a failed repair. Test
   `Agent.DeployRolloutRestartUsesNoBudgetOrIncident`.
2. **Param LKG bound to its config.** The parameter baseline is valid only while
   `lkg_params_hash == spawn_hash`; across a config change `param_diff` is empty. Before, the
   parameter baseline learned under the old config made `param_restore` revert a valid deploy.
3. **Drain grace for downstream deficit.** A downstream node is not blamed while its upstream
   recovered less than 4 x victim_grace_s ago and every one of its outputs is within its
   stale limit at >= 0.8 x expected rate. Before, the backlog a restarted upstream leaves
   was read as the downstream's own fault. Tests `Policy.DrainingDeficitAfterUpstreamRecoveryIsNotBlamed`,
   `Policy.SlowDownstreamAfterUpstreamRecoveryIsStillBlamed`, `Policy.DrainGraceIsBounded`.

The c4 injection probe fix (`d25b13a`, a delayed part that finds no process no longer voids
the injection) is also in force.

## Smokes (seen before this file was written; scratch output, not data)

All `--warm 22 --post 85`, one rep per cell.

- **Smoke 1 (at `4ba10f6`, dtx@10 only, eval set uncommitted):** c2 correct, reg_late correct
  (1 rollback), c3_late fail (predicted), reg_chain_late fail, valid_upstream_dip fail with
  1 unsafe action. The two failures led to fixes 1 to 3 above.
- **Smoke 2 (at `8482a33`, clean build):** dtx@10 correct in c2, reg_late, reg_chain_late and
  valid_upstream_dip, 0 unsafe, 0 wrongful; fail in c3_late (verdict +1 on the silent bad
  config, as predicted). rule@5 and txn2@10, run only on reg_chain_late and
  valid_upstream_dip, were correct in both with 0 unsafe. So reg_chain_late does not
  discriminate between the arms: the fault detector, having learned the 120 config's rate,
  flags the drop to about 55 Hz as `degraded` about 3 s after the 60 start (rule@5 event
  trail), before any arm commits it. It is kept as registered.

rule@5 and txn2@10 were not smoked on reg_late, reg_simul_late, c2 or c3_late; their
predictions below come from the gen study and the mechanism, not from observation.
reg_simul_late was smoked by no arm. No scenario, arm, scorer or prediction changed after
smoke 2.

## Claim

> On the `dtx` fault scenarios, `agent_dtx@10` makes more correct recoveries per block than
> both `agent_txn2@10` and `agent_rule@5`, with zero unsafe actions and zero wrongful
> rollbacks of a legitimate deploy.

Falsified by the kill criteria below. A falsification is published as such.

## Arms

| arm | ranker | H | strikes | generations | behavioral verification | role |
|---|---|---|---|---|---|---|
| `none` | n/a | n/a | n/a | n/a | n/a | guard: scenario must not self-heal |
| `agent_rule@5` | rule | 5 | 1 | no | no | current default |
| `agent_txn2@10` | txn (fault-blind) | 10 | 2 | no | no | best prior arm |
| `agent_dtx@10` | dtx | 10 | 2 (1 on verdict -1) | yes (3, P = 30 s) | yes (W 3 s, 0.85, min 3) | new |

Same detector as the gen study, same K = 3. Params at their defaults
(`agent_dtx_ratio` 0.85, `agent_dtx_window_s` 3, `agent_dtx_min_samples` 3).

## Scenarios (`--set dtx`, 7)

Humanoid sim stack (controller_manager -> /joint_states 250 Hz -> rsp -> /tf, spec 83 Hz;
camera). Deploys rewrite `rsp.yaml` and SIGTERM rsp. The "bad" config, publish_frequency 60,
is real stock-node behaviour: /tf at about 55 Hz, under the 0.85 x 83 = 70.6 Hz contract dtx
checks but above the 0.5 x learned rate the fault detector flags, so the detector alone never
sees it. Its delayed failure is injected: the process is killed (SEGV) 22 s after each start
while that config is live (`markerkill`, conditioned on the config value), so the fault
arrives after rule@5 (H = 5) and txn2 (H = 10) have committed it as LKG. The "valid" config,
publish_frequency 120, is healthy.

| scenario | injection | class | correct end state |
|---|---|---|---|
| nominal | none | control | no intervention |
| reg_late | bad 60 deploy, delayed crash after each start | real delayed config failure | 60 not live at end |
| reg_chain_late | valid 120 deploy, then bad 60 deploy at +18 s, delayed crash | delayed failure after a valid deploy | 120 live, 60 not live |
| reg_simul_late | reg_late on rsp + camera crash (SEGV) at +1 s | simultaneous faults | rsp off 60, camera restarted |
| c2_safe_deploy_twice_late | valid 120 deploy, unrelated crashes 22 s after each start | legitimate deploy, unrelated crash | 120 kept, no revert |
| valid_upstream_dip | valid 120 deploy, controller_manager frozen (SIGSTOP) at +4 s while the deploy is being verified, then 2 unrelated rsp crashes 22 s after start | legitimate deploy, upstream fault + unrelated crashes | 120 kept, no revert, both restored |
| c3_late_cfg_late | silent bad config (publish_frequency 101, no rate signal), delayed crash | registered cost | bad config not live |

VALID (wrongful rollback measured) = c2_safe_deploy_twice_late, valid_upstream_dip.

## Predictions

| scenario | rule@5 | txn2@10 | dtx@10 |
|---|---|---|---|
| reg_late | fail (rolls back to the committed 60) | fail (same) | correct |
| reg_chain_late | correct | correct | correct |
| reg_simul_late | fail | fail | correct |
| c2_safe_deploy_twice_late | correct | correct | correct |
| valid_upstream_dip | correct | correct | correct |
| c3_late_cfg_late | fail | fail | fail (registered cost) |
| nominal | no action | no action | no action |

Predicted: dtx better than each comparator in the 8 reg_late + reg_simul_late blocks, worse
in none; dtx 0 unsafe and 0 wrongful; predicted default decision: see below.

## Metrics

Per arm and per scenario (`score_dtx.py`, rows from `score_gen.gen_row`): correct, strict
restore, restored, repair_ok, unsafe (includes reverting while the wanted config is not live
at the end, and any revert in a safe-deploy scenario), rollbacks, failed rollbacks, act/ep,
downtime_s, ranker ms, healer CPU and RSS. Added: reverts (restart_lkg or param_restore
actions), wrongful (1 for an episode of a VALID scenario with any revert), and the dtx verdict
trace per episode.

## Analysis

Blocks are (scenario, rep), paired across arms. A block with pre_ok = False or no injection
subscribers in any arm is dropped and listed. A fault scenario where `none` restores more than
20% of its episodes leaves the analysis.

- **Primary 1**: dtx@10 - txn2@10 on correct over all fault blocks.
- **Primary 2**: dtx@10 - rule@5 on correct over all fault blocks.
- Both: mean, 95% block bootstrap CI (10 000 resamples), exact sign-flip p (one-sided), Holm
  across the two.
- Per scenario: dtx better / worse block counts against each comparator.
- Wrongful rollbacks per arm over the VALID scenarios.
- Cost: correct per arm in c3_late_cfg_late.
- Safety: unsafe and failed rollbacks per arm; any intervention in nominal.
- Overhead: ranker ms, healer CPU and RSS per arm, descriptive.

Power limit, stated before the run: 24 fault blocks per comparison, of which 8 are predicted
discordant. The smallest one-sided sign-flip p with 8 discordant blocks all in dtx's favour is
1/256. If fewer than 5 blocks are discordant in dtx's favour with none against, the p cannot
reach 0.05 after Holm. This is a mechanism check with 4 reps, not an effect-size estimate, and
the scenarios are built so that the bad config shows a rate symptom.

## Kill criteria

The dtx claim is dead if any holds:

- either primary has CI lo <= 0 or Holm p >= 0.05;
- dtx has any unsafe action or any wrongful rollback;
- dtx is correct in fewer than 3/4 of the blocks of any of reg_late, reg_simul_late,
  c2_safe_deploy_twice_late, valid_upstream_dip.

## Decision rule (default)

`agent_dtx@10` replaces the default `agent_rule@5` only when all hold: the claim survives; dtx
is worse than rule@5 in no block of any scenario; dtx never acts in nominal. Otherwise the
default stays rule@5 and dtx stays opt-in.

Stated now: even a passing result would be 4 reps on one simulated stack, with a symptom the
scenarios were designed to show. A passing result makes dtx the candidate default for the
GO2 Orin observe-only stage, not a proven default for the robot. Any promotion in the code
waits for that evidence and the project owner's approval.

## Procedure

1. `run_episodes.py --set dtx --policies none,agent_rule@5,agent_txn2@10,agent_dtx@10 --reps 4
   --slots 5 --warm 22 --post 85` (7 x 4 x 4 = 112 episodes, about 40 min).
2. One run, no reruns, no commits while it runs. An episode that crashes the harness is
   recorded as failed for that arm.
3. Score with `score_dtx.py RUN --csv docs/v3/results/dtx-recovery.csv --verdicts`, commit the
   CSV and the scorer output under `docs/v3/results/`, write `docs/v3/agent/dtx-results.md`
   with every deviation, failure and negative result.

Scope limits: simulation stack on the dev workstation only. No hardware actuation, no Orin
measurement, no soak. No model, no learned component.
