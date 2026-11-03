# Pre-registration: HELIX rule ranker vs a transactional restart-and-rollback baseline

The commit that adds this file is both the registration and the freeze. The healer
binary, the `txn` scenario set, the injectors and the scorers (`eval/heal/score_agent.py`,
`eval/heal/score_txn.py`) do not change after it. The runs below use exactly this HEAD;
provenance refuses a dirty tree or a binary built from other sources.

## Question

The withheld-family study (`docs/v3/agent/results.md`) found that the rule ranker restores
0.600 more fault episodes per block than v3 `helix` (CI [+0.433, +0.767]). The rule ranker
bundles two things v3 lacked: last-known-good (LKG) rollback (RestartLKG, ParamRestore)
and fault-class-aware selection among repairs. This study asks which of the two carries
the gain, and whether a deterministic transactional supervisor with the same rollback
machinery is as good on harder cases: multi-cause faults, legitimate deploys followed by
unrelated crashes, and delayed config failures that can poison the LKG snapshot.

## Claim

> On the `txn` challenge set, the HELIX default (`agent_rule@5`) and a transactional
> restart-and-rollback supervisor (`agent_txn@10`) differ in correct repairs (restored,
> poisoned config not live at the end, no unsafe action) by less than 0.10 per block
> overall, and the difference that does exist comes from the LKG hold window H, not from
> repair selection. On the withheld families, plain rollback (`agent_txn@5`) reproduces
> at least 80% of the rule ranker's gain over v3 `helix`.

The registered comparator is `agent_txn@10`. The claim is supported by a primary verdict
of "trade-off" or "equivalent" together with null matched-H selection contrasts. It is
falsified by the decision rules below, and a falsification is published as such.

## The baseline (`txn`, `txn2`)

Same healer, detector, verifier, action budget (K = 3 per incident), LKG machinery and
closed action set as the rule ranker. Differences, all deliberate:

- It never reads the fault class: no LifecycleReset, no crash-loop or QoS special cases.
- Candidate order: ParamRestore if a live parameter differs from LKG; RestartLKG first if
  the failed process ran an uncommitted config (after `strikes` consecutive such failures,
  1 for `txn`, 2 for `txn2`); then Restart; then RestartLKG if the disk config is
  uncommitted but the process had not loaded it.
- A config is committed as LKG after the process that loaded it has been healthy for
  `agent_lkg_hold_s` = H seconds (the same rule the rule ranker uses; H is a parameter of
  both).

Mechanism note, written before the run: on a crashing non-lifecycle node (all of C2 and
C3 use `rsp`), `rule` and `txn` (strikes 1) build the same candidate order. At matched H
they can differ only where the fault class matters. The 2x2 below separates selection
(rule vs txn at matched H) from the window (H = 5 vs 10).

## Arms

Run A, `--set txn`:

| Arm | Purpose |
|---|---|
| `none` | fault reality: no healer actions |
| `agent_rule@5` | HELIX default (treatment) |
| `agent_txn@10` | primary comparator: transactional baseline with a conservative hold |
| `agent_rule@10` | window ablation of the treatment |
| `agent_txn@5` | completes the 2x2: selection at matched H = 5 |
| `agent_txn2@10` | two-strike variant: one extra restart before a rollback |
| `agent_oracle@10` | ceiling and positive control: the scenario's known repair ranked first |

Run B, `--set agent_withheld` (attribution of the earlier gain):

| Arm | Purpose |
|---|---|
| `helix` | v3 policy, the earlier study's comparator |
| `agent_rule@5` | the shipped rule ranker |
| `agent_txn@5` | rollback without selection, matched window |

Dose (A7): there is no sham arm, since no arm uses a detector to decide when. Every arm
sees the same detector and the same incidents. Actions per episode (act/ep) and rollbacks
per arm and family are printed and compared, so a gain bought with more actions is
visible.

## Scenarios (Run A)

`nominal`, plus three families. D is the follow-up delay after the fresh process is first
seen.

- **C1 multi-cause**: `c1_latent_cfg_crash` (a bad URDF written to disk, unloaded until an
  unrelated crash restarts the node), `c1_cm_cfg_camera_hang` (bad controllers config plus
  a camera hang), `c1_rsp_param_camera_crash` (a live parameter fault plus a camera
  crash).
- **C2 legitimate deploy, then unrelated crash** at D in {early 4, mid 11, late 22} s: the
  repair is a restart; reverting the deploy is unsafe.
- **C3 delayed config failure** at D in {4, 11, 22} s: every rsp process started with the
  deployed value is killed D s after start. The repair is restart_lkg. Once the bad config
  is committed as LKG, it is poisoned. `c3_late_cfg_late` is registered `restorable=False`
  for any arm, because the bad config is committed by every arm before the crash.

## Calibration (fixed before the freeze, latencies only)

Measured on 1-rep calibration runs, from the deploy TERM to the rsp LKG config commit:
L(5) = 8.23 s, L(10) = 13.23 s (identical across arms to 0.01 s; the healer commits on a
fixed tick). The follow-up delays satisfy early 4 < L(5) < mid 11 < L(10) < late 22, with
at least 2.2 s of margin on each side. `--warm 22` is used for every arm so both the config
and parameter LKG snapshots exist before injection under H = 10 (parameter LKG at about
19 s). Run B keeps the earlier study's warm 12 / post 25, which is valid at H = 5.

## Predictions (derived from the mechanism; the smoke and calibration outcomes, seen before this file was written, agree with every cell they covered)

| Scenario | rule@5 | txn@10 | rule@10 | txn@5 | txn2@10 | oracle |
|---|---|---|---|---|---|---|
| C1 (all three) | correct | correct | correct | correct | correct | correct |
| C2 early | unsafe revert | unsafe revert | unsafe | unsafe | correct | correct |
| C2 mid | correct | unsafe revert | unsafe | correct | correct | correct |
| C2 late | correct | correct | correct | correct | correct | correct |
| C3 early | correct | correct | correct | correct | correct (one extra restart) | correct |
| C3 mid | poisoned, fails | correct | correct | poisoned, fails | correct | correct |
| C3 late | fails | fails | fails | fails | fails | see below |

Primary prediction: rule@5 wins C2 mid, txn@10 wins C3 mid, everything else ties, so the
aggregate is about 0 and the verdict is "trade-off". Selection at matched H: about 0.
Window: H = 10 trades C2 mid for C3 mid. txn2 beats txn on C2 early and mid at a cost of
one extra restart on C3 early and mid. Run B: txn@5 matches rule@5 on every withheld
scenario, so rollback carries all of the gain.

C2 early and C3 early present identical evidence at the first follow-up crash: a config
change, a restart, then a SEGV of the process that ran it, D s later. The deployed values
(120 vs 101 Hz) are both plausible. Any single-strike policy, deterministic or learned,
must get one of the two wrong. Only one more restart on the new config separates them,
which is what `txn2` does.

The oracle ranks the scenario's known repair first at every incident, including the
deploy's own restart. On C3 it therefore reverts the deploy before it can be committed,
so it can succeed on `c3_late_cfg_late`. That is privileged information (it knows which
deploy is bad), not a repair any agent could derive from the evidence. It is reported,
and it does not count toward model reconsideration (below).

## Confounds

- **A1 onset**: injection at warm 22 s, after both LKG snapshots and the healthy-run hold;
  follow-ups are timed from the fresh process, never from episode start.
- **A2 exposure**: every arm runs the same schedule (fixed per scenario and rep), the same
  episode length (warm 22 + post 35 s) and the same detector, so the number of decision
  points is set by the healer's own actions only. act/ep is printed.
- **A3 seeds**: 5 reps per (scenario, arm), paired by (scenario, rep). The aggregate has
  45 fault blocks. At 5 reps a single scenario's exact sign-flip floor is 1/32 one-sided.
  Expected aggregate CI half-width with the predicted 10 discordant blocks is about 0.14,
  wider than the 0.10 margin, which is why the trade-off rule exists.
- **A4 bundling**: rule vs txn differ in fault-class use only; H is varied in the 2x2;
  txn2 differs from txn in the strike count only. The primary comparison (rule@5 vs
  txn@10) bundles selection and H by design, because those are the shipped default and the
  strongest natural baseline. The matched-H secondaries separate them.
- **A5 positive control**: oracle correct >= 0.90 on restorable fault episodes, else the
  run is invalid. Fault reality: `none` restores <= 20% of a fault scenario, else that
  scenario leaves the paired analysis.
- **A6 outcome**: `correct` = the harness probe (an external subscriber, not the healer)
  sees every judged output fresh at the end, the injector reports the bad-config marker not
  live at the end, and no unsafe action (a revert of a legitimate deploy, or any action in
  nominal). Downtime and failed rollbacks are measured from the same probe and event log.
- **A7 dose**: act/ep and rollbacks per arm and family are printed; see Arms.
- **A8 ceiling and floor**: most cells are 0 or 1 by design. The claim is about these
  mechanisms, not a population of faults.
- **A9 selection after seeing data**: H, D and warm were fixed from calibration latencies.
  A 1-rep smoke (3 arms) and a 1-rep calibration (5 arms, 3 scenarios) were run and their
  outcomes were seen; they matched the predictions above. Two changes followed them,
  both before this freeze: the scorer now scores the follow-up repair from the follow-up
  fault onwards (the deploy's own restart was being counted), and the trade-off verdict
  and the matched-H secondaries were added after the smoke showed the predicted
  cancellation. Run B uses the spent withheld families, and `txn` was designed after their
  results were known, so Run B is an attribution analysis and is labelled as such, not as
  a held-out test.
- **A10 leakage**: no learned component is used, trained or tuned. AGENT_PLUMBING episodes
  are not used.

Emulation caveat: the C3 "bad value" is an injector that SEGVs rsp processes started with
it; real `robot_state_publisher` does not crash on 101 Hz. The healer sees only the
process exit, its config hash and its outputs, as it would for a real crash. All nodes are
real ROS 2 processes on Humble; no hardware is actuated.

Exclusions: the smoke, calibration and Run B pilot directories are not study data and are
not scored with the study.

## Metrics

Per arm, per family and per scenario: correct, strict restore, restored, repair_ok (the
first follow-up action verified), unsafe actions, rollbacks, failed rollbacks (a
RestartLKG or ParamRestore that did not verify), LKG commits, act/ep, downtime_s (probe
outage, censored at episode end), ranker ms, healer CPU and RSS.

## Analysis

Blocks are (scenario, rep), paired across arms. A block with pre_ok = False or no
injection subscribers in any arm is dropped and listed.

- **Primary**: agent_rule@5 - agent_txn@10 on correct over the fault blocks. Mean, 95%
  block bootstrap CI (10 000 resamples), exact sign-flip p. Per-scenario discordance: a
  scenario favours an arm when at least 80% of its paired blocks go that way.
- **Verdict**, in order: "trade-off" if at least one scenario favours each arm; "HELIX
  beats txn" if CI lo > 0; "txn beats HELIX" if CI hi < 0; "equivalent" if the CI lies
  inside [-0.10, +0.10] and no scenario favours either arm; otherwise "inconclusive".
- **Secondary** (Holm across five): selection at H = 10 (rule@10 - txn@10), selection at
  H = 5 (rule@5 - txn@5), window for rule (rule@10 - rule@5), window for txn (txn@5 -
  txn@10), two-strike (txn2@10 - txn@10), all on correct. Per-family primary is
  descriptive.
- **Exploratory**: downtime_s and failed rollbacks for the primary pair (lower is better).
- **Safety**: unsafe, failed rollbacks and act/ep per arm x family; any action in nominal.
- **Run B** (`score_txn.py --attrib`): rollback = txn@5 - helix and selection = rule@5 -
  txn@5, on correct and on restored, Holm across the four, and the share of rule - helix
  reproduced by rollback.

## Decision rules

- **Selection matters** if either matched-H selection secondary has Holm p < 0.05 in
  favour of rule, or Run B's selection contrast does. Otherwise the report says the
  measured gains come from rollback and the window, not selection.
- **Claim falsified** if the primary verdict is "HELIX beats txn" or "txn beats HELIX", or
  if Run B's rollback share on restored is below 0.80.
- **HELIX default stays** `agent_rule@5` unless txn@10 or txn2@10 beats it on correct
  with no more unsafe actions; such a change is proposed, not made, in this study.
- **Model reconsideration**: a small LLM or neural policy is reconsidered only if a
  scenario shows an oracle gap >= 0.20 against the best deterministic arm (txn2
  included), and the evidence visible to the agent at the decision distinguishes that
  scenario from one where the oracle's action would be wrong. Registered as
  evidence-identical at the first follow-up crash: c2_X and c3_X for each X. A gap that a
  deterministic change closes (more strikes, keeping more than one LKG generation) is
  reported as a deterministic fix, not as a case for a model.

## Procedure

1. Run A: `run_episodes.py --set txn`, all 7 arms, all 10 scenarios, 5 reps, 5 slots,
   warm 22, post 35 (350 episodes, about 70 min).
2. Run B: `run_episodes.py --set agent_withheld`, arms helix, agent_rule@5, agent_txn@5,
   all 7 scenarios, 4 reps, 5 slots, warm 12, post 25 (84 episodes, about 15 min).
3. One run each, no reruns, no commits while either runs. An episode that crashes the
   harness is recorded as failed for that arm.
4. Score with `score_txn.py` (Run A) and `score_txn.py --attrib` (Run B), commit the raw
   CSVs and scorer output under `docs/v3/results/`, then the post-run audit: recompute
   from the committed CSVs, diff the realized design against this file, print act/ep per
   arm, and record every deviation.
