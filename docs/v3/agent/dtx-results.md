# Results: deployment-aware transactional recovery (`dtx`) vs txn2@10 and rule@5

Pre-registration: [`docs/preregistration/dtx-recovery.md`](../../preregistration/dtx-recovery.md),
frozen at `7e73b24` before any compute. One run, 112 episodes, 112/112 rc = 0. The run
started from a clean tree at that commit (`build.json`: head `7e73b24`, `tree_dirty: false`,
binary built from `7e73b24`, `binary_sources_differ_from_head: false`). It used real ROS 2
processes and the independent probe, with hardware actuation disabled. No reruns. Raw
outputs: [`dtx-recovery.csv`](../results/dtx-recovery.csv) /
[`.txt`](../results/dtx-recovery.txt). No model was added. The HELIX default is still
`agent_rule@5`.

## Bottom line

1. **Registered verdict: the dtx claim is dead.** Neither primary clears Holm.
   - dtx - txn2 on correct = +0.208, CI [+0.042, +0.375], one-sided p = 0.031, Holm p = 0.0625.
   - dtx - rule@5 = +0.125, CI [+0.000, +0.250], one-sided p = 0.125, Holm p = 0.125.
2. **Default unchanged (registered rule).** rule@5 stays the default and dtx stays opt-in.
3. **The safety and correctness requirements held in every episode.**
   - dtx: 0 unsafe actions, 0 wrongful rollbacks, 0 failed rollbacks, 0 actions in nominal.
   - dtx was correct 4/4 in each of `reg_late`, `reg_simul_late`, `c2_safe_deploy_twice_late` and `valid_upstream_dip`.
   - dtx was never worse than either comparator in any block.
   - The comparators also had 0 unsafe actions and 0 wrongful rollbacks.
4. **The comparison was confounded by the shared fault detector, which the pre-registration
   wrongly said would never see the bad config.**
   - The detector flagged the bad `publish_frequency 60` config as `degraded` about 3 s after
     it started in 14 of the 24 `reg_late` + `reg_simul_late` episodes, across all arms
     (dtx 5/8, rule 5/8, txn2 4/8).
   - Whether it fired varied between reps of the same cell, and it decided most outcomes.
   - It does not change the registered verdict. It does mean the primaries partly measure
     this detector noise and not dtx.
5. **Exploratory, not registered: when the detector missed the bad config, only dtx recovered.**
   - Of the 8 dtx episodes in these two scenarios, the detector stayed silent or fired once
     in 4. dtx's behavioral verdict marked the config `failed` (good 0, bad 3, /tf 55/83)
     in all 4, and dtx reverted on the first delayed crash.
   - In the comparator episodes where the detector did not trigger that arm's revert (txn2:
     fewer than two `degraded`; rule@5: none), the bad config was committed and kept:
     txn2 0/5 correct, rule@5 0/3.
   - This split was drawn after seeing the data. It is evidence that the mechanism works,
     not a test of it.
6. **The registered cost held as predicted.** Every arm failed `c3_late_cfg_late`
   (0/4 each), the silent bad config with no rate signal. In every episode dtx gave it a
   `verified` verdict. Behavioral verification can only see a fault that shows in what it
   measures.

## Per arm (24 fault blocks each)

| arm | correct | unsafe | wrongful | rollbacks | failed rb | act/ep | downtime s | censored |
|---|---|---|---|---|---|---|---|---|
| none | 0/24 | 0 | 0 | 0 | 0 | 0.00 | 84.94 | 24 |
| agent_rule@5 | 17/24 | 0 | 0 | 9 | 0 | 3.42 | 16.43 | 3 |
| agent_txn2@10 | 15/24 | 0 | 0 | 7 | 0 | 3.92 | 24.12 | 5 |
| agent_dtx@10 | **20/24** | 0 | 0 | 12 | 0 | 3.54 | **10.62** | 0 |

Overhead: ranker 0.0026 ms per decision for all arms; healer CPU 0.014 and RSS about
18.6 MB per agent arm (none: 0.012, 17.5 MB). Descriptive only.

## Per scenario, correct

| scenario | rule@5 | txn2@10 | dtx@10 | registered prediction (rule / txn2 / dtx) |
|---|---|---|---|---|
| reg_late | 3/4 | 1/4 | 4/4 | fail / fail / correct |
| reg_chain_late | 4/4 | 4/4 | 4/4 | correct / correct / correct |
| reg_simul_late | 2/4 | 2/4 | 4/4 | fail / fail / correct |
| c2_safe_deploy_twice_late | 4/4 | 4/4 | 4/4 | correct / correct / correct |
| valid_upstream_dip | 4/4 | 4/4 | 4/4 | correct / correct / correct |
| c3_late_cfg_late | 0/4 | 0/4 | 0/4 | fail / fail / fail (cost) |
| nominal (no action) | 4/4 | 4/4 | 4/4 | no action |

Every scenario's guard passed: `none` restored 0/4 in each fault scenario. No block was
dropped (every block had pre_ok, and every injection found its subscribers).

Discordant blocks:
- dtx vs txn2: 5, all in dtx's favor (reg_late 3, reg_simul_late 2).
- dtx vs rule@5: 3, all in dtx's favor (reg_late 1, reg_simul_late 2).
- No block favored a comparator.

## What decided each `reg_*` episode

`degraded` is the existing fault detector's incident on rsp after the bad deploy, given as
the times it fired. `failed` is dtx's behavioral verdict, given as its time. `revert` is the
first `restart_lkg`.

| scenario rep | rule@5 | txn2@10 | dtx@10 |
|---|---|---|---|
| reg_late r0 | degraded 25.4, revert 25.4, ok | degraded 25.4 once, no revert, **fail** | degraded 25.4 once; failed 29.6; revert on crash 47.7, ok |
| reg_late r1 | degraded 25.3, revert, ok | no degraded, **fail** | degraded 25.4 + 28.7, two-strike revert 28.7, ok |
| reg_late r2 | no degraded, **fail** | degraded 25.4 + 28.7, revert, ok | degraded x2, revert 28.7, ok |
| reg_late r3 | degraded 25.3, revert, ok | no degraded, **fail** | degraded x2, revert 28.7, ok |
| reg_simul_late r0 | degraded, revert 25.4, ok | no degraded, **fail** | degraded x2, revert 29.4, ok |
| reg_simul_late r1 | degraded, revert 25.3, ok | degraded x2, revert 29.5, ok | no degraded; failed 27.3; revert on crash 44.3, ok |
| reg_simul_late r2 | no degraded, **fail** | no degraded, **fail** | no degraded; failed 27.3; revert on crash 44.3, ok |
| reg_simul_late r3 | no degraded, **fail** | degraded x2, revert 29.5, ok | no degraded; failed 27.3; revert on crash 44.3, ok |

Read honestly:
- The behavioral verdict decided 4 of dtx's 8 wins here (reg_late r0, reg_simul_late r1 to r3).
- The other 4 dtx wins came from the same detector luck the comparators had. In those, dtx
  reverted by its verdict-0 path (txn2's two strikes) before the verdict had its 3 samples,
  so dtx and txn2 behaved the same.
- Of the 5 blocks where dtx beat txn2, two were decided by the verdict (reg_late r0,
  reg_simul_late r2). The other three were decided by the detector firing in the dtx
  episode but not in the txn2 one.
- In `reg_chain_late` the detector fired reliably for every arm, as the smoke showed. dtx
  and txn2 needed two strikes there (revert at about 46.7 s) where rule@5 needed one
  (43.4 s). dtx's downtime was 25.4 s against rule@5's 22.1 s. This cost does not show in
  correct.

## Kill criteria and decision rule (as registered)

- Primary 1: CI lo +0.042 > 0, but Holm p 0.0625 >= 0.05: **kill met**.
- Primary 2: CI lo +0.000 <= 0, and Holm p 0.125 >= 0.05: **kill met**.
- dtx unsafe 0, wrongful 0: not met.
- dtx correct 4/4 in each of the four named scenarios: not met.
- Decision rule: the claim does not survive, so **rule@5 stays the default** and dtx stays
  opt-in. dtx was worse than rule@5 in 0 blocks and never acted in nominal, but those
  conditions do not matter once the claim is dead.

## Deviations and errors (all disclosed)

1. **Mechanism claim in the pre-registration was wrong.**
   - The pre-registration said the bad 60 config's /tf rate (about 55 Hz) sits above the
     detector's `degraded` threshold, "so the detector alone never sees it".
   - In practice the detector flagged it in 14/24 `reg_*` episodes, about 3 s after the start.
   - The cause is not diagnosed. ASSUMPTION: the rate window still spans the restart gap or
     the startup transient after the deploy restart (`envelope.hpp` counts the open gap).
     The two-config smokes did not show it because rule@5 and txn2 were not smoked on
     `reg_late`.
   - This noise made rule@5 and txn2 better than predicted on `reg_late` and
     `reg_simul_late`, and cut the discordant blocks from 8 predicted to 5 and 3.
2. **The pre-registered power statement was off by one.**
   - It said fewer than 5 favorable discordant blocks could not reach 0.05 after Holm.
   - In fact 5 blocks give a one-sided p of 1/32 and a Holm p of 0.0625, so 6 are needed.
   - This run had exactly 5 against txn2, so the error matters here: with the power limit
     stated correctly, the result would have been known to be underpowered at 5.
3. **Source edits during the run.** Observe-mode changes to `healer_node.cpp`, `spec.*`,
   the tests and the launch file were edited in the tree while the run was going.
   - They were not built and not committed.
   - The run binary is the one recorded in `build.json` (sha256 `37b6132c...`, built from `7e73b24`).
   - The Python harness, injector and scorer were not touched.
   - Scoring ran on the frozen scorer, which is unchanged in the tree.

## What this establishes, and what it does not

Established, on this simulated stack:
- Behavioral evidence keyed to a deployment can tell a bad deploy from a legitimate one.
  dtx kept the valid deploy in 8/8 VALID episodes, including the ones with an upstream
  freeze and two unrelated crashes.
- It also catches a delayed config failure that rollback-by-timing misses: 4/4 of the
  episodes where the detector did not catch it.
- It did both with 0 wrongful rollbacks.
- This is the gen study's open problem, choosing when to revert, answered on these
  scenarios by measured health and not by who deployed.

Not established:
- A significant advantage over either comparator: the registered test failed.
- Any result on a silent bad config: 0/4, the registered cost.
- Anything on real robot components: the study used one simulated stack and 4 reps.
- Anything on the Orin: not measured.

Not worth another large run before the evidence improves:
- The next limit is the detector noise that decided half the `reg_*` outcomes, not dtx's
  decision rule.
- A rerun with more reps would mostly re-measure that noise.
- The next step on this line is a desk diagnosis of the 3 s `degraded` flag after a
  deploy restart. After that, real component signals on the GO2 under observe mode
  (`docs/go2/observe.md`).
