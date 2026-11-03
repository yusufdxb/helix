# Pre-registration: versioned config snapshots (`gen`) against txn2@10 and rule@5

The commit that adds this file is both the registration and the freeze. The healer
binary, the `gen` scenario set, the injectors (`eval/heal/humanoid/inject.py`) and the
scorers (`eval/heal/score_agent.py`, `eval/heal/score_gen.py`) do not change after it.
The run below uses exactly this HEAD; provenance refuses a dirty tree.

## Background (the negative result this builds on, unchanged)

The transactional study (`docs/preregistration/txn-baseline.md`, results in
`docs/v3/agent/`) found that the recovery advantage of the HELIX agent comes from
last-known-good (LKG) rollback and the hold window H, not from intelligent repair
selection: a fault-blind two-strike supervisor (`agent_txn2@10`) matched or beat the
rule ranker (Run A: txn2@10 40/45 correct, 0 unsafe; rule@5 29/45, 5 unsafe), and the
selection contrasts were null. That result stands and is not re-litigated here.

The remaining oracle gap was `c3_late`: a bad config that fails later than the commit
latency L(H) is committed as the LKG before it fails, so rollback restores the poison.
This study tests one deterministic fix for that gap. No model, no learned component.

## The mechanism (`agent_gen@10`)

`gen` is txn2 (fault-blind order, two strikes) plus versioned snapshots:

- Each managed node keeps a ring of at most 3 config generations
  `{id, hash, files, trusted, rejected}`; the LKG is the newest non-rejected generation.
- **Transactional promotion**: a new generation is committed only after H seconds of
  health with every upstream managed node also healthy, the files are staged in
  `g<id>.tmp`, re-hashed against the spawn hash, renamed, and the source is re-checked
  under the lock. Eviction drops the oldest rejected generation first.
- **Trust**: a generation becomes trusted after `agent_gen_probation_s` (30 s) of health
  with upstream healthy. A trusted generation is never stepped back from.
- **Step-back**: on the second strike of a recurring fault while the running generation is
  on probation and a previous generation exists, RestartLKG marks the running generation
  rejected, pins the node, and restarts on the previous generation (`pin` event
  `step back gX -> gY`). Strikes are counted per config hash, so a new deploy starts clean.

Five unit tests cover the decision logic (`test_agent.cpp`, `Gen.*`; 78/78 pass at this
HEAD).

Known, registered limitations (by construction, not discovered after the run):

1. A failure later than the probation window P = 30 s after start is treated as
   unrelated to the config (no step-back).
2. A valid deploy followed by two unrelated crashes inside P is evidence-identical to a
   recurring delayed config fault. `gen` reverts it, which is unsafe. This is the cost case.
3. The first generation (G0, committed during warm-up) is untrusted at injection
   (warm 22 s < P).

## Claim

> On the `gen` set, `agent_gen@10` makes more correct repairs per fault block than
> `agent_txn2@10` (95% block bootstrap CI lo > 0), by repairing poisoned-LKG cases that
> txn2 cannot (`c3_late_cfg_late`, `c4_chain_late`, `c5_simul_late`), at the cost of
> unsafe reverts in `c2_safe_deploy_twice_late`, with no change elsewhere.

Falsified by the kill criterion below. A falsification is published as such.

## Arms

| arm | ranker | H | strikes | generations | role |
|---|---|---|---|---|---|
| `none` | n/a | n/a | n/a | n/a | guard: scenario must not self-heal |
| `agent_rule@5` | rule | 5 | 1 | no | current default |
| `agent_txn2@10` | txn (fault-blind) | 10 | 2 | no | best prior arm, primary comparator |
| `agent_gen@10` | txn (fault-blind) | 10 | 2 | yes | new |

`gen` and `txn2` differ only in the generation ring, probation and step-back, so the
primary contrast isolates versioned snapshots. Same detector (401b4b1), same K = 3.

## Scenarios (`--set gen`, 8, all restorable by some correct policy)

| scenario | injection | correct end state |
|---|---|---|
| nominal | none | no intervention |
| c3_late_cfg_mid | bad rsp config, crash 11 s after each start (D < L(10)) | bad config not live |
| c3_late_cfg_late | bad rsp config, crash 22 s after each start (D > L(10), poisons LKG) | bad config not live |
| c2_safe_deploy_crash_late | valid deploy, one unrelated crash 22 s after start | deploy kept, no revert |
| c2_safe_deploy_twice_late | valid deploy, two unrelated crashes 22 s after each start | deploy kept, no revert |
| c4_chain_late | valid deploy (120), then bad deploy (101) at +18 s, crash 22 s after each 101 start | 120 live at end, 101 not live |
| c1_cm_cfg_camera_hang | corrupt controller config + camera hang (from `txn`) | both restored |
| c5_simul_late | c3_late_cfg_late on rsp + camera crash at +1 s | both restored, bad config not live |

## Smoke (seen before this file was written; dirty build, scratch output, not data)

One rep of gen@10 and txn2@10 on c3_late_cfg_late, c4_chain_late, c2_safe_deploy_twice_late:
gen correct 1, 1, 0 (unsafe 1); txn2 correct 0, 0, 1. In c4 the step-back `g3 -> g2`
landed 62 s after injection (inside post 85 s) and the 120 process was live at the end.
These smoke episodes are excluded from the analysis.

## Predictions

| scenario | rule@5 | txn2@10 | gen@10 |
|---|---|---|---|
| c3_late_cfg_mid | correct | correct | correct |
| c3_late_cfg_late | fail (poisoned LKG) | fail | correct |
| c2_safe_deploy_crash_late | correct or unsafe | correct | correct |
| c2_safe_deploy_twice_late | unsafe likely | correct | unsafe (cost case) |
| c4_chain_late | fail | fail | correct |
| c1_cm_cfg_camera_hang | correct | correct | correct |
| c5_simul_late | fail | fail | correct |

Predicted primary mean gen - txn2 about +0.43 (3 of 7 scenarios gained, 1 lost).
Predicted default decision: unchanged, because gen is unsafe in the cost case.

## Metrics

Per arm and per scenario (`score_gen.py`): correct, strict restore, restored, repair_ok,
unsafe, rollbacks, failed rollbacks, LKG commits, act/ep, downtime_s (censored at episode
end), ranker ms, healer CPU and RSS.

- correct = restored, poisoned marker not live at the end, unsafe = 0, and where the
  scenario has a markerwatch (c4) the wanted config is live at the end.
- unsafe = score_agent's count (any revert in a `safe_deploy` scenario, any action on a
  node outside root, fallback and collateral) plus 1 when the arm reverted and the
  wanted config is not live at the end (rolled back past a valid deploy).

## Analysis

Blocks are (scenario, rep), paired across arms. A block with pre_ok = False or no
injection subscribers in any arm is dropped and listed. A fault scenario where `none`
restores more than 20% of its episodes leaves the paired analysis.

- **Primary**: agent_gen@10 - agent_txn2@10 on correct over the fault blocks: mean, 95%
  block bootstrap CI (10 000 resamples), exact sign-flip p, and per-scenario discordance.
- **Secondary** (Holm across three): gen@10 - rule@5 on correct; txn2@10 - rule@5 on
  correct (replication of Run A); gen@10 - txn2@10 on downtime_s (lower is better).
- **Safety**: unsafe and failed rollbacks per arm; any intervention in nominal.
- **Overhead**: ranker ms, healer CPU and RSS per arm, descriptive.

## Kill criterion

The gen claim is dead if the primary CI includes 0, or gen is correct in fewer than 3/4
of the c3_late_cfg_late blocks or of the c4_chain_late blocks.

## Decision rule (default)

`agent_gen@10` replaces the default `agent_rule@5` only when all hold: the claim
survives the kill criterion; gen - rule@5 has CI lo > 0 and Holm p < 0.05; gen has no
more unsafe actions than either comparator; gen never acts in nominal. Otherwise the
default is unchanged and gen stays opt-in. Given registered limitation 2, the expected
outcome is "unchanged".

## Procedure

1. `run_episodes.py --set gen --policies none,agent_rule@5,agent_txn2@10,agent_gen@10
   --reps 4 --slots 5 --warm 22 --post 85` (8 x 4 x 4 = 128 episodes, about 46 min).
2. One run, no reruns, no commits while it runs. An episode that crashes the harness is
   recorded as failed for that arm.
3. Score with `score_gen.py RUN --csv`, commit the CSV and scorer output under
   `docs/v3/results/`, write `docs/v3/agent/gen-results.md` with every deviation and failure.

Scope limits: simulation stack on the dev workstation only. No hardware actuation, no
Orin measurement, no multi-hour soak.
