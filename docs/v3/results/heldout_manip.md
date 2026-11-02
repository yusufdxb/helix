# Held-out results: MANIP topology, 3 reps (pre-registered, run once)

Pre-registration: [`docs/preregistration/2026-11-02-helix-v3-heldout-manip.md`](../../preregistration/2026-11-02-helix-v3-heldout-manip.md),
frozen at commit `687a036`. The run used that commit with a clean tree; the
commit hash is written to `eval/heal/out/heldout_manip.commit` at launch.
The only MANIP run before this one was the allowed nominal-only check (one
episode per policy, all RESTORED, 0 decisions).

```
python3 eval/heal/run_episodes.py --set manip --reps 3 --out eval/heal/out/heldout_manip
python3 eval/heal/score.py eval/heal/out/heldout_manip --csv eval/heal/out/heldout_manip.csv
```

180 episodes: 15 scenarios x 4 policies x 3 reps. Every healer exited with rc=0,
and 0 stray processes remained. Per-episode rows are in `heldout_manip.csv`.
R = restored, S = SAFE_STOP, F = failed. uN = N unnecessary decisions, bN = N
episodes ending with a runaway or leak still burning.

## Study 1: repair policy

```
scenario                  helix           none            respawn         watchdog
cpu_runaway               RRR             RRR b3          RRR b3          RRR b3
crash_exit                RRR             FFF             RRR             RRR
crash_loop_backup         RRR             FFF             FFF             RRR u12
crash_loop_nobackup       SSS             FFF             FFF             SSS
crash_segv                RRR             FFF             RRR             RRR
degrade                   RRR             FFF             FFF             FFF
degrade_lifecycle         RRR             FFF             FFF             FFF
endpoint_loss             RRR             FFF             FFF             RRR
endpoint_loss_lifecycle   RRR             FFF             FFF             RRR
hang                      RRR             FFF             FFF             RRR u3
hang_latch                RRR             FFF             FFF             RRR u9
hang_lifecycle            RRR             FFF             FFF             RRR
leak                      RRR             RRR b3          RRR b3          RRR
nominal                   RRR             RRR             RRR             RRR
qos_mismatch              SSS             FFF             FFF             SSS u24

policy    n  restored  safe  failed  diag_ok  decisions  unneeded  futile  burning  lat_med_s  outage_mean_s  cpu%  rss_mb
none      45         9     0      36    0/42           0        0       0        6       0.00          21.10   0.7    15.3
respawn   45        15     0      30    6/42         180        0     141        6       0.67          17.73   0.8    15.3
watchdog  45        33     6       6   18/42         120       48      18        3       2.25           9.05   0.7    15.4
helix     45        39     6       0   42/42          60        0       6        0       2.35           5.72   0.7    15.3

PRIMARY helix - watchdog: RESTORED diff +6 over 14 fault scenarios (2 non-tied blocks, p-value floor 0.25); exact sign-flip p = 0.25; unnecessary helix 0 vs watchdog 48; dropped pairs 0
KILL CRITERION (diff < 3 or more unnecessary): not met, claim survives
```

**Decision rule.** HELIX restored 39 of 45 episodes and the watchdog 33, a
difference of +6, which is at least the required 3. HELIX made 0 unnecessary
decisions and the watchdog 48. The kill criterion is not met, so the claim
survives on this topology.

**Significance is not claimed.** All 6 extra restorations come from 2 scenarios,
`degrade` and `degrade_lifecycle`. A node that keeps publishing at 25 to 40% of its
rate is never stale, so the watchdog never acts on it. The other 12 fault scenarios
tie. At the registered unit (the scenario) the exact sign-flip p is 0.25, its
floor with 2 non-tied blocks. The pre-registration predicted this.

**Prediction check.** The registration predicted the gap would come from rate
degradation and from fewer collateral restarts. The first part held. The second
shows up only in the dose, not in the outcome: the watchdog's 48 unnecessary
restarts (of starved downstream nodes in `crash_loop_backup`, `hang`, `hang_latch`
and `qos_mismatch`) cost no RESTORED episodes. They did raise its mean outage:
9.05 s against 5.72 s for HELIX.

### Audit (post-run, study-gate step 6)

- **Positive controls, all passed.**
  - `nominal` was RESTORED with 0 decisions under all 4 policies.
  - `crash_segv` was RESTORED 3/3 under `respawn`.
  - Every restorable fault except `cpu_runaway` and `leak` was FAILED 3/3 under
    `none`.
  - `cpu_runaway` and `leak` ended `burning` 3/3 under `none`, as the amended
    control requires.
- **A1 onset.** `pre_ok` was True in all 180 episodes. The probe confirmed delivery
  (`inject_subs > 0`) in every fault episode. Nominal has nothing to inject. 0 pairs
  were dropped.
- **A7 dose.** Decisions per arm: HELIX 60, watchdog 120, respawn 180, none 0.
  HELIX won with half the watchdog's interventions, not more of them.
  - HELIX dose by scenario: 3 in every scenario (one root repair per episode) except:

    | Scenario | HELIX decisions |
    |---|---|
    | `crash_loop_backup` | 12 (restarts, fallback, then the latched detector as declared collateral) |
    | `crash_loop_nobackup` | 9 (two restarts to establish the loop, then SAFE_STOP) |
    | `hang_latch` | 6 (camera, then the latched detector) |
  - The 6 HELIX futile repairs are the two restarts per `crash_loop_nobackup`
    episode that establish the crash loop. SAFE_STOP is published 1.65 s after the
    fault (rep 0, `events.jsonl`).
- **Deviations from the registration: none in Study 1.**
- **Secondary results, not headline.**

  | Metric | HELIX | Watchdog |
  |---|---|---|
  | Median recovery latency | 2.35 s | 2.25 s |
  | Mean outage | 5.72 s | 9.05 s |
  | Episodes left burning a resource | 0 | 3 |

  - HELIX's latency includes the 1 s confirmation window for non-exit symptoms.
  - The 3 watchdog burning episodes are `cpu_runaway`: a CPU hog is neither stale
    nor over RSS, so the watchdog leaves it running.
  - Healer overhead was 0.7% of one core and 15.3 MB RSS in every arm. Every arm
    runs the same process, and `none` only observes.

## Study 2: learned diagnosis vs rules

```
python3 eval/heal/learned.py --train eval/heal/out/dev_nav --test eval/heal/out/heldout_manip
python3 eval/heal/learned.py --train eval/heal/out/dev_nav --test eval/heal/out/heldout_manip --llm qwen3:4b-instruct
python3 eval/heal/learned.py --train eval/heal/out/dev_nav --test eval/heal/out/heldout_manip --llm qwen3:8b
```

| Arm | A: decision point (n=42) | B: false alarms, fault-free snapshots | C: median detection |
|---|---|---|---|
| rules (HELIX) | **42/42** | 0 unnecessary decisions in nominal | 0.71 s (median act latency) |
| GBDT, trained on DEV | 36/42 | 2/3504 (0.06%) | 0.61 s (hang_latch, hang_lifecycle never detected) |
| qwen3:4b-instruct, zero-shot, schema-constrained | 21/42 | not measured | 0.36 s per call (inference only) |
| qwen3:8b, zero-shot, schema-constrained | 19/42 | not measured | 0.49 s per call (inference only) |
| oracle | 42/42 | 0 | n/a |

Both Ollama models ran entirely on the GPU (`ollama ps`: 100% GPU).

**Integration rule.** A learned diagnoser is integrated only if all three hold:
A is at least rules + 2, B is no higher than the rules', and C is no slower.
The rules scored 42/42, the ceiling, so A could not be met by any arm. The GBDT
also fails B (2 against 0) and passes only C. Neither LLM reaches the rules on A.
**The deterministic rules stay in the healer, and the learned result is
published as a negative.**

What the learned arms show:

- **The GBDT generalises to the new topology on most faults.** Trained only on
  the NAV topology, it named the right (node, fault) on MANIP in 36/42 cases.
- **It misses all 6 hang episodes on two nodes.** On `hang_latch` (camera) and
  `hang_lifecycle` (arm_planner) it outputs no fault, with probability at most
  0.11. It catches `hang` on the tracker 3/3. The cause was not investigated: the
  freeze forbids tuning on held-out data.
- **It detects faster on symptom faults.** Per scenario, GBDT detection against
  HELIX act latency (range over 3 reps):

  | Scenario | GBDT | Rules |
  |---|---|---|
  | `endpoint_loss`, both variants | 0.31 to 0.51 s | 0.66 to 1.01 s |
  | `degrade`, both variants | 0.66 to 0.95 s | 1.06 to 1.21 s |
  | `cpu_runaway` | 0.85 to 1.05 s | 2.41 to 2.46 s |
  | `leak` | 1.45 to 1.55 s | 3.41 to 3.91 s |
  | crashes (4 scenarios) | 0.40 to 0.56 s | 0.002 to 0.055 s |
  | `hang` | 0.70 to 0.76 s | 0.60 to 0.61 s |

  The rules are fastest on crashes because they see `waitpid` directly. They are
  slower on symptoms because they wait out a confirmation window. The GBDT's
  lower median comes from this mix. It is bought with false alarms the rules do
  not raise and with 6 hangs never detected. A hybrid (rules for exits, a learned
  detector for symptoms) is the obvious next experiment and is not tested here.
- **The 4B LLM mostly answers "degraded".** It labelled every hang and every
  lifecycle endpoint loss as `degraded`. It got qos_mismatch, leak, degrade and
  gripper endpoint loss right. Decision-point accuracy flatters any arm, because
  the rules chose that snapshot (see the registered selection-bias note), and the
  LLM still scored half.
- **The 8B LLM is not better than the 4B.** It scored 19/42. It named the right
  node on every hang but called it `degraded`. It answered "no fault" on CPU
  runaway, both crash loops and 2 of 3 leaks, and called the segfaulted `joints`
  node `degraded`. Scaling the zero-shot model did not close the gap. One untested
  explanation: the rules read `waitpid`, lifecycle state and per-output rates
  directly, while the model has to infer them from the snapshot text.

### Deviations in Study 2

- The registration lists metric B (false alarms) and metric C (detection time) as
  comparisons for the "learned diagnoser". `learned.py` measures B and C for the
  GBDT only, and A for the LLMs. Both LLMs already fail A, so the missing B and C
  cannot change the integration decision. They are reported as not measured.
