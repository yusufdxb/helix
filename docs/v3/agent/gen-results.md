# Results: versioned config snapshots (`gen`) vs txn2@10 and rule@5

Pre-registration: [`docs/preregistration/gen-snapshots.md`](../../preregistration/gen-snapshots.md),
frozen at `029b41c` before any compute. One run, 128 episodes, clean tree at that commit
(`build.json`: head `029b41c`, `tree_dirty: false`), real ROS 2 processes, the fixed
detector (`401b4b1`), independent probe for recovery, hardware actuation disabled. No
reruns. Raw outputs: [`gen_snapshots.csv`](../results/gen_snapshots.csv) /
[`.txt`](../results/gen_snapshots.txt). No model was added. The HELIX default is still
`agent_rule@5`.

## Bottom line

1. **Registered verdict: the gen claim is dead.** The kill criterion is met because the
   primary CI includes 0: gen@10 - txn2@10 on correct = +0.167, 95% block bootstrap CI
   [-0.125, +0.417], one-sided p = 0.194 (n = 24 blocks). The predicted effect was about
   +0.43.
2. **Default unchanged (registered rule).** rule@5 stays the default and gen stays opt-in.
   This holds under every analysis below, because gen made 4 unsafe reverts and both
   comparators made 0.
3. **The mechanism works where it was meant to.** gen repaired every poisoned-LKG block
   that ran: `c3_late_cfg_late` 4/4 (txn2 0/4, rule 0/4) and `c5_simul_late` 4/4 (txn2 0/4,
   rule 0/4). In `c4_chain_late` gen was 4/4 and txn2 0/4, but those blocks were dropped
   (see Deviations).
4. **The registered cost is real.** In `c2_safe_deploy_twice_late` gen reverted a valid
   deploy in 4/4 blocks (4 unsafe). txn2 and rule kept the deploy in 4/4. This is
   registered limitation 2: two unrelated crashes inside probation look exactly like a
   recurring delayed config fault, and the healer has no evidence to tell them apart.
5. **The earlier negative result is preserved.** Recovery advantage comes from rollback
   and its timing, not from selection. gen keeps txn2's fault-blind order; the gains and
   the losses both come from which snapshot is rolled back to.

## Totals (fault scenarios, 28 episodes per arm, before the block drop)

| arm | correct | unsafe | restored | downtime_s (mean) | censored | rollbacks | failed rollbacks |
|---|---|---|---|---|---|---|---|
| agent_gen@10 | 24/28 | 4 | 28/28 | 1.46 | 0 | 24 | 0 |
| agent_txn2@10 | 16/28 | 0 | 28/28 | 2.70 | 0 | 8 | 0 |
| agent_rule@5 | 12/28 | 0 | 24/28 | 13.31 | 4 | 4 | 0 |
| none | 0/28 | 0 | 0/28 | 84.97 | 28 | 0 | 0 |

`restored` without `correct` means the node is up but the poisoned config is still the
running one (it crash-loops on its own schedule). Downtime does not penalise that state,
which is why txn2 and rule have low downtime on c3_late and c5 while being wrong.

## Per scenario (correct out of 4)

| scenario | rule@5 | txn2@10 | gen@10 | predicted (rule / txn2 / gen) |
|---|---|---|---|---|
| c1_cm_cfg_camera_hang | 4 | 4 | 4 | correct / correct / correct |
| c2_safe_deploy_crash_late | 4 | 4 | 4 | correct or unsafe / correct / correct |
| c2_safe_deploy_twice_late | 4 | 4 | **0** (4 unsafe) | unsafe likely / correct / unsafe |
| c3_late_cfg_mid | **0** (restored 0/4) | 4 | 4 | correct / correct / correct |
| c3_late_cfg_late | 0 | 0 | 4 | fail / fail / correct |
| c4_chain_late (dropped) | 0 | 0 | 4 | fail / fail / correct |
| c5_simul_late | 0 | 0 | 4 | fail / fail / correct |
| nominal | 4 | 4 | 4 | no arm acts |

Two predictions were wrong, both for rule@5:

- **rule@5 failed `c3_late_cfg_mid` (0/4, predicted correct).** Trace of rep 0: the bad
  config crashes 11 s after each start, but H = 5 commits the LKG about 8 s after a
  restart, so the bad config was already the LKG at 30.3 s. Each later crash offered only
  `restart`, the action budget (K = 3) ran out at 55.5 s and the healer fell back to
  SAFE_STOP with no backup. For H = 5 the "mid" delay is already a poisoned-LKG case. This
  is a real rule@5 failure, recorded as such.
- **rule@5 was correct on `c2_safe_deploy_twice_late` (predicted unsafe likely).** The
  short hold window commits the valid deploy before the first crash, so restarts are
  enough and nothing is reverted.

## Registered analysis (c4 blocks dropped, n = 24)

- Guard: `none` restored 0/4 in every fault scenario; no scenario excluded.
- **Primary** gen@10 - txn2@10 (correct): +0.167 [-0.125, +0.417], p one-sided 0.194.
  Discordance: gen better in c3_late 4/4 and c5 4/4, txn2 better in c2_twice 4/4, ties
  elsewhere.
- Secondary (Holm across three):
  - gen@10 - rule@5 (correct): +0.333 [+0.042, +0.625], p 0.0384, Holm p 0.115.
  - txn2@10 - rule@5 (correct, replication of Run A): +0.167 [+0.042, +0.333], p 0.0625.
    Direction replicates; not significant at this n.
  - gen@10 - txn2@10 (downtime, lower better): +0.135 s [-0.037, +0.319], p 0.93. No
    downtime gain.
- Safety: unsafe gen 4, txn2 0, rule 0, none 0. Failed rollbacks 0 in every arm. No arm
  acted in nominal.
- Kill: MET (primary CI includes 0). Default: unchanged.

## Overhead (descriptive)

| arm | ranker ms | healer CPU (fraction of a core) | healer RSS MB |
|---|---|---|---|
| agent_gen@10 | 0.0027 | 0.014 | 18.6 |
| agent_txn2@10 | 0.0027 | 0.014 | 18.6 |
| agent_rule@5 | 0.0028 | 0.015 | 18.3 |
| none | n/a | 0.013 | 17.3 |

The generation ring adds no measurable ranker, CPU or memory cost on the dev workstation.
Orin was not measured.

## Deviations

1. **All four `c4_chain_late` blocks were dropped by the registered rule.** The `none` arm
   reported `inject_subs = 0` in all four c4 reps (other arms 2; `none` in other scenarios
   1 or 2). The drop rule removes a block when any arm has no injection subscribers, so c4
   left the paired analysis. The `none` injector still logged `deploy_set` and
   `deploy_TERM`, so the deploys did happen and the zero looks like a probe counting
   artefact, but the cause is not determined. The rule was applied as registered; nothing
   was rerun. This is the reason the registered primary has n = 24 instead of 28.
2. The c4 second deploy is at +18 s. This was set before the freeze and is in the
   registered scenario table; listed here because it changed after the smoke run.
3. Smoke: one dirty-build rep of gen and txn2 on c3_late, c4, c2_twice was seen before the
   pre-registration was written. It is disclosed there and excluded here.

## Post-hoc (not registered, c4 kept, n = 28)

Computed after seeing the registered result, to show what the drop cost. It does not
change the verdict.

- gen@10 - txn2@10: +0.286 [+0.036, +0.536], p 0.0384.
- gen@10 - rule@5: +0.429 [+0.143, +0.679], p 0.0059.
- txn2@10 - rule@5: +0.143 [+0.036, +0.286], p 0.0625.

Even here the registered default rule would not promote gen: gen has 4 unsafe actions
against 0 for both comparators.

## What this means for HELIX

- Versioned snapshots with probation fix the poisoned-LKG gap that txn2 cannot fix
  (8/8 gen vs 0/8 txn2 on c3_late and c5; 4/4 vs 0/4 on c4 outside the analysis).
- They do it by trading one failure mode for another: a valid deploy followed by two
  unrelated crashes gets reverted. Without evidence about the deploy itself (who deployed
  it, whether it was intended), the healer cannot separate the two cases. This is a limit
  of the available evidence, not a tuning problem.
- gen stays opt-in (`agent_ranker: gen`). rule@5 stays the default. The txn2@10 switch proposed
  after Run A is still only a proposal.

## Follow-ups

1. Find why the `none` arm reports `inject_subs = 0` in c4 and fix the probe, then
   pre-register a confirmatory run before any c4 claim is made.
2. Deploy-legitimacy evidence for the cost case (an explicit deploy intent or operator
   acknowledgement that marks a generation trusted), pre-registered against c2_twice.
3. rule@5's c3_late_cfg_mid failure shows H = 5 poisons the LKG for 11 s delays; this is
   one more data point for the txn2@10 proposal, not a new decision.

Scope: simulation stack on the dev workstation only. No hardware actuation, no Orin
measurement, no soak.
