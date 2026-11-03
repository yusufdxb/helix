# Results: HELIX rule ranker vs a transactional restart-and-rollback baseline

Pre-registration: [`docs/preregistration/txn-baseline.md`](../../preregistration/txn-baseline.md),
frozen at `6319f97` before any compute. Both runs used that commit with a clean tree, real
ROS 2 processes (controller_manager, robot_state_publisher, a camera node), an independent
probe for recovery, and hardware actuation disabled. Raw outputs:
[`txn_baseline.csv`](../results/txn_baseline.csv) / [`.txt`](../results/txn_baseline.txt) (Run A) and
[`txn_attrib.csv`](../results/txn_attrib.csv) / [`.txt`](../results/txn_attrib.txt) (Run B).
No model was added. The HELIX default is still `agent_rule@5`.

## Bottom line

1. **Selection adds nothing measurable over plain transactional rollback.** At a matched
   hold window, rule minus txn is +0.000 [0, 0] (H = 10) and -0.022 (H = 5), Holm p = 1 for
   both. On the scenarios of the earlier
   withheld study (Run B), plain rollback reproduces 100% of the rule ranker's gain over
   `helix` (+0.500 [+0.292, +0.708], Holm p = 0.00098) and selection adds +0.000 (Holm
   p = 1). The earlier recovery improvement came from adding last-known-good rollback.
2. **The primary contrast is a trade-off, not a win.** rule@5 - txn@10 on correct =
   -0.022, 95% CI [-0.156, +0.111], one-sided p = 0.73. rule@5 wins C2 mid (5/5), txn@10
   wins C3 mid (5/5). The difference between the two arms is the LKG hold window, not the
   ranking: H = 5 commits a fresh config sooner, which protects a legitimate deploy from a
   later revert and poisons the LKG when the deploy itself fails late.
3. **The strongest arm is deterministic: two-strike rollback (`txn2@10`).** 40/45 correct,
   0 unsafe actions, 0 failed rollbacks. txn2 - txn = +0.222 [+0.111, +0.356], Holm p =
   0.0049, the only significant secondary. Against the HELIX default (exploratory, not
   registered): txn2@10 - rule@5 = +0.244 [+0.133, +0.378] correct, -0.111 [-0.200,
   -0.022] unsafe, -4.1 s [-7.7, -1.0] downtime, +0.244 [+0.067, +0.422] actions per
   episode.
4. **Default decision (registered rule): change proposed, not made.** txn2@10 beats
   rule@5 on correct with fewer unsafe actions (0 vs 5), which under the pre-registered
   rule triggers a proposal to switch the default. This study does not make the switch.
5. **No case for a model.** The only oracle gap is `c3_late_cfg_late` (+1.00). That gap
   needs privileged knowledge of which deploy is bad, the scenario is evidence-identical
   to `c2_safe_deploy_crash_late` at the decision, and keeping more than one LKG
   generation is a deterministic fix candidate. Under the registered rule this is not a
   model case.
6. **Negative found along the way: a detector-confirmation bug.** It caused the only
   unpredicted miss (rule@5 on `c1_rsp_param_camera_crash` rep 2). See below.

## Run A: 7 arms x 10 scenarios x 5 reps (350 episodes, 350 rc = 0, 0 stray processes)

Fault scenarios only (n = 45 blocks per arm). `correct` = strict and no unsafe action;
`strict` = restored and the poisoned-config marker is not live at the end; `restored` =
the independent probe saw the outputs healthy at the end; `repair_ok` = the verified
repair was the right class for the root cause (scored separately from `correct`).

| Arm | correct | strict | restored | repair_ok | unsafe | failed rollbacks | act/ep | downtime s (censored) |
|---|---|---|---|---|---|---|---|---|
| oracle@10 (ceiling) | 45 | 45 | 45 | 45 | 0 | 0 | 1.67 | 1.82 (0) |
| **txn2@10** | **40** | 40 | 45 | 30 | **0** | 0 | 2.67 | 2.60 (0) |
| txn@10 | 30 | 40 | 45 | 30 | 10 | 0 | 2.22 | 1.96 (0) |
| rule@10 | 30 | 40 | 45 | 30 | 10 | 0 | 2.27 | 2.02 (0) |
| txn@5 | 30 | 35 | 40 | 30 | 5 | 0 | 2.44 | 5.97 (5) |
| rule@5 (HELIX default) | 29 | 34 | 39 | 29 | 5 | 0 | 2.42 | 6.68 (6) |
| none | 0 | 0 | 0 | 0 | 0 | 0 | 0.00 | 34.98 (45) |

Nominal: 0 actions in every arm. Guards: oracle correct on restorable episodes 40/40 =
1.000 (PASS, need >= 0.90); `none` restores 0/5 on every fault scenario. Ranker time about
0.003 ms per decision, healer CPU about 0.014 cores, RSS about 18.5 MB in every arm.

### Per scenario (correct out of 5)

| Scenario | rule@5 | txn@10 | rule@10 | txn@5 | txn2@10 | oracle |
|---|---|---|---|---|---|---|
| c1_latent_cfg_crash | 5 | 5 | 5 | 5 | 5 | 5 |
| c1_cm_cfg_camera_hang | 5 | 5 | 5 | 5 | 5 | 5 |
| c1_rsp_param_camera_crash | **4** | 5 | 5 | 5 | 5 | 5 |
| c2 early (D = 4 s) | 0 (unsafe revert) | 0 (unsafe) | 0 (unsafe) | 0 (unsafe) | 5 | 5 |
| c2 mid (D = 11 s) | 5 | 0 (unsafe) | 0 (unsafe) | 5 | 5 | 5 |
| c2 late (D = 22 s) | 5 | 5 | 5 | 5 | 5 | 5 |
| c3 early | 5 | 5 | 5 | 5 | 5 (3 actions, repair_ok 0) | 5 |
| c3 mid | **0 (never restored)** | 5 | 5 | **0 (never restored)** | 5 | 5 |
| c3 late | 0 | 0 | 0 | 0 | 0 | 5 (privileged) |

Notes on the negatives:

- **C2 early defeats every single-strike arm.** A legitimate deploy followed 4 s later by
  an unrelated crash looks exactly like a bad deploy. All four single-strike arms revert a
  good deploy (unsafe) in 5/5. Only the extra restart of txn2 separates the two.
- **C3 mid at H = 5 is worse than predicted.** The registration predicted "poisoned,
  fails". The realized outcome is stronger: rule@5 and txn@5 never restore rsp at all
  (35 s downtime, censored, 4 actions per episode). The bad config was committed as LKG,
  so restart_lkg reinstalls the failing config.
- **C3 late fails for every deterministic arm.** Every deterministic arm leaves rsp
  restored at the end (5/5, marker live), but none uses the correct repair, because the bad
  config is committed before the crash. The oracle's success here comes from privileged
  information, as registered.
- **txn2 is not free.** It averages 2.67 actions per episode against 2.22 for txn, its
  repair_ok is 30/45 (an extra restart precedes the rollback), and C1 downtime is higher
  (8.27 s vs 6.23 s on c1_cm_cfg_camera_hang).
- **Downtime favours txn@10 over the default.** rule@5 - txn@10 = +4.72 s [+1.67, +8.36]
  (exploratory). It is driven by the censored never-restored C3 mid episodes at H = 5.

### Registered contrasts

| Contrast | Mean (correct) | 95% CI | Holm p |
|---|---|---|---|
| **Primary** rule@5 - txn@10 | -0.022 | [-0.156, +0.111] | p1 = 0.73 (not Holm) |
| Selection, H = 10 (rule@10 - txn@10) | +0.000 | [+0.000, +0.000] | 1 |
| Selection, H = 5 (rule@5 - txn@5) | -0.022 | [-0.067, +0.000] | 1 |
| Window, rule (rule@10 - rule@5) | +0.022 | [-0.111, +0.156] | 1 |
| Window, txn (txn@5 - txn@10) | +0.000 | [-0.133, +0.133] | 1 |
| Two-strike (txn2@10 - txn@10) | **+0.222** | **[+0.111, +0.356]** | **0.0049** |

Families (descriptive): C1 -0.067, C2 +0.333 [+0.133, +0.600], C3 -0.333 [-0.600,
-0.133]. Verdict by the registered order: **trade-off**.

## Run B: attribution on the earlier study's scenarios (84 episodes, 84 rc = 0, 0 stray processes)

Arms `helix` (restart only, no rollback), `agent_rule@5` (the default), `agent_txn@5`
(plain rollback, same hold window). Scenarios: nominal, a legitimate deploy
(`safe_deploy_rsp`), two parameter faults (w1) and three bad-config faults (w2), 4 reps.

| Arm | correct (fault, n = 24) | repair_ok | unsafe | failed rollbacks | act/ep (all) | downtime s (censored) |
|---|---|---|---|---|---|---|
| agent_rule@5 | 24/24 | 24/24 | 0 | 0 | 1.57 | 2.98 (0) |
| agent_txn@5 | 24/24 | 24/24 | 0 | 0 | 1.57 | 3.17 (0) |
| helix | 12/24 | 4/24 | 0 | 0 | 1.86 | 13.32 (12) |

- rule@5 and txn@5 made the same actions per episode, scenario by scenario.
- helix fails all three w2 bad-config scenarios (0/4 each, about 25 s censored downtime):
  restarting reinstalls the bad config.
- helix scores `correct` on the w1 parameter faults by restarting (the restart reloads
  launch parameters), but not `repair_ok` (0/4 and 0/4).
- No arm acted on nominal; the legitimate deploy was kept by every arm.

| Contrast (Holm across four) | Mean | 95% CI | Holm p |
|---|---|---|---|
| Rollback, txn@5 - helix, correct | +0.500 | [+0.292, +0.708] | 0.00098 |
| Rollback, txn@5 - helix, restored | +0.500 | [+0.292, +0.708] | 0.00098 |
| Selection, rule@5 - txn@5, correct | +0.000 | [+0.000, +0.000] | 1 |
| Selection, rule@5 - txn@5, restored | +0.000 | [+0.000, +0.000] | 1 |

Share of the rule@5 - helix gain reproduced by plain rollback: **1.00** on correct and on
restored (registered falsifier: < 0.80). The registered claim, that the earlier gain comes
from rollback rather than selection, survives.

## Detector-confirmation bug (found during the audit)

The single rule@5 miss on `c1_rsp_param_camera_crash` (rep 2) is not a selection
difference. In that episode the parameter injection cut `/tf` from 83 Hz to 5 Hz, and the
healer marked rsp symptomatic from 22.9 s to the end. It never opened an incident, so the
outcome was FAILED with 35 s of censored downtime.

Cause, reproduced offline against the built policy library: at 5 Hz the output age climbs
from 0 to about 0.2 s every period. The adaptive stale limit is 0.1 s. The diagnosis
therefore flips between Hang (age above the limit) and Degraded (schedule deficit) about
every 0.1 s. Each flip resets the pending-fault timer, and the 0.1 s adaptive confirmation
never elapses. A scratch probe with gap 0.200 s and limit 0.100 s produced no decision in
30 s, as did a gap of 0.198 s. A gap of 0.21 s confirmed in 0.41 s, and a limit of 0.098 s
confirmed after 16.2 s. That knife edge matches the 3.3 to 7.9 s detection latency in the
other reps. The bug is in the shared detector, so it affects every policy that uses it.
It was not fixed during the study, and every number above comes from the pre-fix
detector at `6319f97`. The fix (keep the pending timer when the diagnosis moves between
the output symptoms Hang, EndpointLoss and Degraded) and a regression test land in a
separate post-study commit.

## Post-run audit

- Recomputed from the committed CSVs: per-arm correct, unsafe and failed rollbacks, the
  primary (-0.022), selection at H = 10 (+0.000), two-strike (+0.222), txn2@10 - rule@5
  (+0.244), Run B rollback (+0.500) and selection (+0.000), and every cell of the
  per-scenario table reproduce exactly from the two CSVs alone.
- Realized design vs registration: both runs as registered (arms, scenarios, reps, slots,
  warm and post), one run each, no reruns, no commits while running. Deviations: none in
  procedure. Unpredicted outcomes: the rule@5 c1 miss (detector bug above), and C3 mid at
  H = 5 never restored (stronger than the predicted "poisoned, fails"). Every other cell
  of the predictions table matched.
- A7 dose: no detector-timed sham applies (every arm sees the same detector and
  incidents). Actions per episode are printed per arm and family; the arm that wins
  (txn2) uses more actions, not fewer, and that cost is reported above.
- Exclusions: none (no block had pre_ok = False or zero injection subscribers).

## What this does not show

- All failures are injected on a desk ROS 2 stack. The delayed config failure in C3 is
  emulated (a marker-triggered kill D s after start), not a real config bug. The real
  Orin onboard computer was not measured.
- The trade-off is tied to the chosen follow-up delays (4, 11, 22 s) relative to the
  measured LKG commit latencies (8.2 s at H = 5, 13.2 s at H = 10). A different delay
  distribution moves the balance between C2 mid and C3 mid.
- n = 5 reps per scenario. Within-scenario outcomes were near deterministic, so the CIs
  mostly reflect scenario composition, not run-to-run noise.
