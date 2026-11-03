# Pre-registration: confirm `gen` on poisoned LKG, and test hash-bound deploy acknowledgement

The commit that adds this file is both the registration and the freeze. The healer
binary, the `gen2` scenario set, the injector (`eval/heal/humanoid/inject.py`) and the
scorers (`eval/heal/score_agent.py`, `score_gen.py`, `score_gen2.py`) do not change after
it. The run below uses exactly this HEAD; provenance refuses a dirty tree.

## Background (unchanged results this builds on)

- Run A (`docs/preregistration/txn-baseline.md`): the recovery advantage of HELIX comes from
  last-known-good (LKG) rollback and its timing, not from intelligent repair selection. This
  is not re-litigated.
- The gen study (`docs/preregistration/gen-snapshots.md`, results
  `docs/v3/agent/gen-results.md`): the registered gen claim is dead (gen@10 - txn2@10 =
  +0.167, CI [-0.125, +0.417], n = 24). gen repaired every poisoned-LKG block that ran
  (c3_late 4/4, c5 4/4, c4 4/4 but dropped) and reverted a valid deploy in
  `c2_safe_deploy_twice_late` 4/4 (4 unsafe). The default stayed rule@5.

## What changed since that run (two fixes, both from its follow-ups)

1. **c4 drop, cause now determined** (`d25b13a`). In the `none` arm the first c4 deploy
   SIGTERMs rsp and nothing restarts it, so the second deploy (`@18`) finds no process and
   records `hits = 0`. The composite rule required every initial part to hit, so the whole
   episode reported `inject_subs = 0` and the registered drop rule removed all four c4
   blocks. Now a delayed part that finds no process is listed in `delayed_missed` and does not
   void the injection; an undelayed miss still does. Regression tests:
   `eval/heal/tests/test_inject.py` (two of them fail on the old rule).
2. **Hash-bound deploy acknowledgement** (`04969af`, healer param `agent_gen_ack_dir`). A
   deploy tool, after its own acceptance check, appends the FNV-1a 64 hash of each accepted
   config file to `<agent_gen_ack_dir>/<node>.ack` (16 hex digits per line). A non-rejected
   generation whose every config file hash is listed becomes trusted at once (`lkg_trust`
   event `gN ack`), so gen never steps back from it; a second crash only restarts it. An ack
   never un-rejects a generation, and with no ack file the healer behaves exactly as gen.
   Unit tests: `Gen.AckParsesHexLinesOnly`, `Gen.AckMustCoverEveryFile`.
3. **FNV basis fix** (`663eed4`, found by the smoke below). The healer's `file_hash` and
   `files_hash` used the offset basis 1469598103934665603, the FNV-1a 64 basis
   14695981039346656037 with its last digit dropped. Internal LKG comparisons only compare
   healer hashes with each other and were unaffected, but an ack written with standard
   FNV-1a could never match, so (2) was inert. Now a shared `fnv1a64` with the published
   parameters, test `Gen.AckHashIsStandardFnv1a64` (published vectors for "", "a", "foobar").
   The same commit adds a diagnostic `lkg_ack` event, emitted only when an ack file changes,
   that lists the hashes each generation needs. 81/81 healer gtests and 7/7 eval tests pass
   at this HEAD.

The acknowledgement is evidence from outside the healer about the deploy itself, which the
gen study showed the healer cannot get from crash timing. It is only as good as the
acceptance check behind it: a bad config whose fault shows after the check passes gets
acked, and then gen behaves like txn2 and keeps the poison. That case is in the run as a
registered cost.

## Claims

> **A (confirmatory).** On the poisoned-LKG scenarios (`c3_late_cfg_late`, `c4_chain_late`,
> `c5_simul_late`), `agent_gen@10` makes more correct repairs per block than
> `agent_txn2@10`.

> **B.** In the c2 cost case with the valid deploy acknowledged
> (`c2_safe_deploy_twice_late_ack`), `agent_genack@10` makes more correct repairs per block
> than `agent_gen@10`, with no unsafe action.

Both are falsified by the kill criteria below. A falsification is published as such.

## Arms

| arm | ranker | H | strikes | generations | reads acks | role |
|---|---|---|---|---|---|---|
| `none` | n/a | n/a | n/a | n/a | n/a | guard: scenario must not self-heal |
| `agent_rule@5` | rule | 5 | 1 | no | no | current default |
| `agent_txn2@10` | txn (fault-blind) | 10 | 2 | no | no | best prior arm |
| `agent_gen@10` | txn (fault-blind) | 10 | 2 | yes | no | gen study arm, unchanged |
| `agent_genack@10` | txn (fault-blind) | 10 | 2 | yes | yes | new |

`genack` is `gen` plus `-p agent_gen_ack_dir:=<episode>/stack/ack`. Only the injector's
`ack` part writes there, so in a scenario without one genack must behave as gen (checked as
IDENTITY below). Same detector (`401b4b1`), same K = 3, same probation P = 30 s.

## Scenarios (`--set gen2`, 7)

| scenario | injection | correct end state |
|---|---|---|
| nominal | none | no intervention |
| c3_late_cfg_late | bad rsp config (101), crash 22 s after each start (poisons the LKG at H = 10) | bad config not live |
| c4_chain_late | valid deploy (120), bad deploy (101) at +18 s, crash 22 s after each 101 start | 120 live at end, 101 not live |
| c5_simul_late | c3_late_cfg_late on rsp + camera crash at +1 s | both restored, bad config not live |
| c2_safe_deploy_twice_late | valid deploy (120), two unrelated crashes 22 s after each start, no ack | deploy kept, no revert |
| c2_safe_deploy_twice_late_ack | the same, with the 120 file acked at +15 s | deploy kept, no revert |
| c3_late_cfg_late_falseack | c3_late_cfg_late with the bad 101 file acked at +15 s | bad config not live |

The first five are the gen-study scenarios, unchanged. The ack lands at +15 s, after gen
commits the deploy as a generation (about +12 s at H = 10) and before the first delayed
crash (about +23 s).

## Smoke (seen before this file was written; dirty build, scratch output, not data)

Arms gen@10 and genack@10, scenarios `c2_safe_deploy_twice_late_ack` and
`c3_late_cfg_late_falseack`, one rep each, `--warm 22 --post 85`.

- **Smoke 1 (at `04969af` + uncommitted eval changes): failed.** genack behaved exactly as
  gen: unsafe revert in c2 ack (both arms 0/1 correct, 1 unsafe), correct in falseack (both
  1/1). The ack part ran (hits 1, file written, param passed), and the ack landed after the
  g2 commit, but no `lkg_trust gN ack` appeared. A one-episode diagnostic with the `lkg_ack`
  event showed the ack held `cc6c361370bba746` while g2 needed `140082ec51e2848c`; a
  standalone reproducer of the healer loop gave `44bd8ad473cd9906` for "a" against the
  standard `af63dc4c8601ec8c`. Cause: the truncated offset basis, fixed in `663eed4`.
- **Smoke 2 (at `5df27c8`, dirty only by this file): as predicted.** genack c2 ack: `lkg_ack`
  then `lkg_trust g2 ack`, no rollback, correct 1/1, unsafe 0. gen c2 ack: rollback past the
  valid deploy, correct 0/1, unsafe 1. genack falseack: bad 101 acked and trusted, kept
  live, correct 0/1 (the registered cost). gen falseack: rolled back, correct 1/1.

Neither smoke is data. No scenario, arm, scorer or prediction changed after smoke 2.

## Predictions

| scenario | rule@5 | txn2@10 | gen@10 | genack@10 |
|---|---|---|---|---|
| c3_late_cfg_late | fail | fail | correct | correct |
| c4_chain_late | fail | fail | correct | correct |
| c5_simul_late | fail | fail | correct | correct |
| c2_safe_deploy_twice_late | correct | correct | unsafe | unsafe (no ack, same as gen) |
| c2_safe_deploy_twice_late_ack | correct | correct | unsafe | correct |
| c3_late_cfg_late_falseack | fail | fail | correct | fail (registered cost) |
| nominal | no action | no action | no action | no action |

Predicted A: gen better than txn2 in 15/15 POISON blocks. Predicted B: genack better than
gen in 5/5 ack blocks, genack unsafe 0 there. Predicted default decision: unchanged,
because genack is still unsafe in `c2_safe_deploy_twice_late` (a deploy that was not
acked) and both txn2 and rule@5 are not.

## Metrics

Per arm and per scenario (`score_gen2.py`, rows from `score_gen.gen_row`): correct, strict
restore, restored, repair_ok, unsafe, rollbacks, failed rollbacks, act/ep, downtime_s
(censored at episode end), ranker ms, healer CPU and RSS. correct and unsafe are defined
exactly as in the gen study (`score_gen.py`, unchanged).

## Analysis

Blocks are (scenario, rep), paired across arms. A block with pre_ok = False or no injection
subscribers in any arm is dropped and listed. A fault scenario where `none` restores more
than 20% of its episodes leaves the analysis.

- **Primary A**: gen@10 - txn2@10 on correct over the POISON blocks.
- **Primary B**: genack@10 - gen@10 on correct over the `c2_safe_deploy_twice_late_ack` blocks.
- Both: mean, 95% block bootstrap CI (10 000 resamples), exact sign-flip p (one-sided), Holm
  across A and B.
- **Secondary** (Holm across three, all fault blocks): genack - txn2; genack - rule@5;
  gen - txn2 (replication of the gen study primary, now with c4 counted).
- **Cost**: correct and unsafe per arm in `c3_late_cfg_late_falseack`.
- **Identity**: discordant genack vs gen blocks in the scenarios without an ack part.
- **Safety**: unsafe and failed rollbacks per arm; any intervention in nominal.
- **Overhead**: ranker ms, healer CPU and RSS per arm, descriptive.

Power limit, stated before the run: B has 5 blocks, so the smallest possible one-sided
sign-flip p is 1/32 = 0.031. B can pass only if genack is better in all 5 blocks and worse
in none. This is a mechanism check, not a precise effect estimate.

## Kill criteria

- **A** is dead if its CI includes 0, its Holm p >= 0.05, or gen is correct in fewer than
  3/4 of the blocks of any POISON scenario.
- **B** is dead if its CI includes 0, its Holm p >= 0.05, or genack has any unsafe action in
  `c2_safe_deploy_twice_late_ack`.

## Decision rule (default)

`agent_genack@10` replaces the default `agent_rule@5` only when all hold: A and B survive;
genack - rule@5 has CI lo > 0 and Holm p < 0.05; genack has no more unsafe actions than
either txn2@10 or rule@5; genack never acts in nominal. Otherwise the default is unchanged
and gen and genack stay opt-in. Expected outcome: unchanged.

## Procedure

1. `run_episodes.py --set gen2 --policies none,agent_rule@5,agent_txn2@10,agent_gen@10,agent_genack@10
   --reps 5 --slots 5 --warm 22 --post 85` (7 x 5 x 5 = 175 episodes, about 64 min).
2. One run, no reruns, no commits while it runs. An episode that crashes the harness is
   recorded as failed for that arm.
3. Score with `score_gen2.py RUN --csv`, commit the CSV and scorer output under
   `docs/v3/results/`, write `docs/v3/agent/gen-ack-results.md` with every deviation and failure.

Scope limits: simulation stack on the dev workstation only. No hardware actuation, no Orin
measurement, no multi-hour soak. No model, no learned component.
