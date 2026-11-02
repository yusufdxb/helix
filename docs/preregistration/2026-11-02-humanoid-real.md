# Pre-registration: HELIX healer on a real ROS 2 humanoid stack

Date: 2026-11-02. Frozen code: the commit that adds this file. Healer code, spec
parameters, scenario lists, the stack generator, the injector and both scoring scripts
are not changed after it.

## What was already seen

The developmental set `scenarios.py:HUMANOID` (13 scenarios) was run as 1-rep smoke tests
while three healer changes were made: core-dump crash detection, fresh-message repair
verification, and the rule that helix backs off only after a young death on a node
without a backup. Every number from that set is developmental and is reported as such.

The held-out set `scenarios.py:HUMANOID_HELDOUT` (9 scenarios, same stack) is defined in
this commit and has never run. It is the only confirmatory evidence.

## Stack

`eval/heal/humanoid/make_stack.py`, all stock ROS 2 Humble packages, nothing written for
HELIX:

- controller_manager (ros2_control with mock hardware, 23 joints, /joint_states at 250 Hz),
  with the stock spawner as a `then` helper.
- robot_state_publisher (/tf at 83 Hz). It has no backup and is critical.
- cam2image at 30 Hz, with a 15 Hz backup.

Faults act on the real PIDs from outside the healer (`eval/heal/humanoid/inject.py`).
Hardware actuation is not involved.

## Claim

On held-out faults of a real ROS 2 humanoid stack, the helix policy:

- restores at least 3 more fault episodes than a conventional watchdog, out of 18 (6 fault scenarios with a root, 3 reps);
- makes no more unnecessary interventions than the watchdog;
- shortens the longest topic blackout per restorable fault episode, with a 95% CI of the
  paired mean difference entirely below 0.

## Arms

| Arm | Purpose |
|---|---|
| helix | Treatment: adaptive envelopes, diagnosis, verification, fallback |
| watchdog | Named comparator: restart on exit or stale output, fixed backoff, budget then fallback |
| respawn | Launch `respawn=true` equivalent. Positive control: every one-shot crash must be RESTORED by it, or the pipeline is broken |
| none | No-treatment control: what the faults do unhealed |
| helix_v3params | Ablation: helix rules with the v3 fixed parameters (no adaptive envelope, no fast repair) |

There is no oracle arm, because the claim is about a whole repair system, not a single
prediction. Diagnosis correctness is reported per episode against labels fixed in
`scenarios.py`.

## Run plan (one run, no reruns)

1. Held-out run: `--set humanoid_heldout`, all 5 arms, 3 reps, 3 slots. 9 x 5 x 3 =
   135 episodes. Each episode is 7 s warm-up then 25 s after injection. Injection
   schedules are seeded per (scenario, rep), so every arm sees the same schedule.
2. Developmental run (labelled developmental, never confirmatory): `--set humanoid`,
   helix and watchdog only, 3 reps. 13 x 2 x 3 = 78 episodes.

No commits are made while either run is in progress. If the runner itself dies part way
through, only the missing episodes are rerun, and that is reported as a deviation.

## Analysis

**P1 (restoration and intervention).** `score.py` primary test, helix minus watchdog:
- The outcome is the RESTORED count. Blocks are fault scenarios, summed over reps.
- The test is an exact one-sided sign-flip.
- Unnecessary interventions are summed per arm.
- With 6 fault-scenario blocks, the p-value floor is 2^-k for k non-tied blocks (at best 2^-6 = 0.016).
  P1 is therefore judged on its kill criterion, not on significance.

**P2 (blackout).** `paired_blackout.py`, helix minus watchdog, on every restorable fault
block (scenario, rep): 5 restorable scenarios with a root x 3 = 15 blocks. It reports:
- the median and mean paired difference in blackout_s;
- a seeded percentile bootstrap 95% CI of the mean;
- the exact two-sided sign-flip p.

**Multiplicity.** Holm correction across P1 and P2 at alpha 0.05 for any significance
statement.

**Secondary (exploratory, labelled as such).** All reported per arm:
- diagnosis accuracy;
- act latency;
- decisions (the dose);
- futile repairs and left_burning;
- SAFE outcome on crashloop_rsp;
- healer CPU and RSS;
- helix vs helix_v3params;
- all developmental-set numbers.

## Kill criteria (numbers, stated before the run)

- **P1 dead:** if the RESTORED diff is < 3, or helix makes more unnecessary interventions
  than the watchdog.
- **P2 dead:** if the upper bound of the bootstrap 95% CI of the paired mean blackout
  difference is >= 0.

A dead claim is published as a negative in the README and the vault, with the numbers.

**Prior expectation, written down so it cannot move.**
- One-shot crashes should tie on RESTORED: both arms restart them.
- The only likely RESTORED gap is degrade_rsp (watchdog has no rate rule), so P1 may well
  die at diff 3 or less.
- P2 is where a difference is expected.

## Confounds

| Check | Answer |
|---|---|
| A1 onset | Faults are injected at 7 s, mid-episode, after the envelopes have learned. The nominal and near-miss episodes measure false alarms; any decision in them is unnecessary. |
| A2 exposure | Every arm runs the same episode length, injection time and seeded fault schedule. |
| A3 seeds | 3 reps. The P1 floor is stated above. P2 has 15 paired blocks, two-sided floor 2^-14. |
| A4 bundling | helix differs from watchdog in detectors, diagnosis, backoff and verification at once. The claim is about the system. helix_v3params isolates the parameter changes only. |
| A5 positive control | The respawn arm on term_rsp and sigkill_cm must be RESTORED. If not, the run is void. |
| A6 outcome | The outcomes are whether the topic is restored to at least 0.8 of its nominal rate, and the longest gap consumers saw. These are system outcomes, not proxies. This is software only, with no robot. |
| A7 dose | Decisions per arm are printed. More restarts can only help an arm's RESTORED count, so a helix win with fewer decisions is not a dose artefact. A helix win with more decisions is flagged as one. |
| A8 ceiling | One-shot crashes are expected at the ceiling for every healing arm, and that is stated. |
| A9 selection | The developmental set was tuned on. The held-out set is defined here and is run once. |
| A10 leakage | Envelopes learn online only from pre-fault healthy data within each episode. Nothing is fit across episodes. |

Known runner limits:
- 3 parallel slots share one host, each pinned to 4 cores, with ROS_LOCALHOST_ONLY and
  distinct domain IDs.
- Latency for the `ctl:` fault includes ros2 CLI startup for every arm alike.
- deactivate_fwd has no root node (the process stays healthy, a controller is switched off).
  It is outside P1 and P2. Any decision in it is reported as an exploratory secondary.
