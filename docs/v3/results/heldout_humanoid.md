# Held-out results: real ROS 2 humanoid stack, 3 reps (pre-registered, run once)

Pre-registration: [`docs/preregistration/2026-11-02-humanoid-real.md`](../../preregistration/2026-11-02-humanoid-real.md),
frozen at commit `9e66b17` (14:32). The held-out run went from 14:33 to 14:57 and the
developmental run ended at 15:11, both on that commit. The next commit (`4764f52`,
post-freeze engineering) landed at 15:30. The runner did not write the commit hash into
its output directory, so this timing is the provenance record.

The stack is stock ROS 2 Humble, nothing written for HELIX:
- `ros2_control` with mock hardware (23 joints, /joint_states at 250 Hz) and the stock spawner;
- `robot_state_publisher` (/tf at 83 Hz, no backup);
- `cam2image` at 30 Hz, with a 15 Hz backup.

Faults act on the real PIDs from outside the healer. No hardware is involved.

```
python3 eval/heal/run_episodes.py --set humanoid_heldout --reps 3 --out OUT
python3 eval/heal/score.py OUT --csv heldout_humanoid.csv
python3 eval/heal/paired_blackout.py heldout_humanoid.csv
```

The held-out run had 135 episodes: 9 scenarios x 5 arms x 3 reps. Every episode exited with rc=0. Per-episode rows are in
`heldout_humanoid.csv`. Key: R = restored, S = SAFE_STOP, F = failed, uN = N unnecessary
decisions.

## Held-out (confirmatory)

```
scenario                  helix           helix_v3params  none            respawn         watchdog
crashloop_rsp             SSS             SSS             FFF             RRR             SSS
deactivate_fwd            RRR             RRR             RRR             RRR             RRR
degrade_rsp               RRR             RRR             FFF             FFF             FFF
flaky_rsp                 RRR             RRR             FFF             RRR             RRR
hang_camera               RRR             RRR             FFF             FFF             RRR
nearmiss_rsp              RRR             RRR             RRR             RRR             RRR
nominal                   RRR             RRR             RRR             RRR             RRR
sigkill_cm                RRR             RRR u3          FFF             RRR             RRR
term_rsp                  RRR             RRR             FFF             RRR             RRR

policy            n  restored  safe  failed  diag_ok  decisions  unneeded  futile  outage_mean_s  blackout_med_s  cpu%  rss_mb
none              27         9     0      18    0/18           0        0       0          24.68          24.995   1.3    16.8
respawn           27        21     0       6    9/18         233        0       0           8.40           0.148   1.3    16.8
watchdog          27        21     3       3   12/18          29        0       0           8.72           0.403   1.3    16.8
helix             27        24     3       0   17/18          29        0       0           4.38           0.259   1.2    16.8
helix_v3params    27        24     3       0   18/18          32        3       6           6.00           0.966   0.9    16.8
```

**P1 (restoration, unnecessary interventions).** helix restored 3 more episodes than the watchdog, all of them in degrade_rsp, where the watchdog has no rate rule.
- helix made 0 unnecessary interventions; the watchdog also made 0.
- Only one block is non-tied, so the p-value floor is 0.5, and p = 0.5.
- The kill criterion (diff < 3, or more unnecessary interventions) is not met.

The claim survives at exactly the threshold. As the pre-registration predicted, it is not significant.

**P2 (longest blackout per restorable fault episode).** The test paired helix and watchdog on 15 restorable fault blocks.
- Median difference: -0.016 s. Mean difference: -0.142 s.
- 95% bootstrap CI: [-0.267, -0.021].
- Exact two-sided sign-flip p = 0.0477. helix was shorter in 10 of 15 blocks.
- The kill criterion (CI upper bound >= 0) is not met, so the claim survives.

**Multiplicity (Holm, alpha 0.05).** The smallest p is 0.0477, which is above 0.025, so neither claim is significant after correction. Both claims survive their pre-registered kill criteria, and that is all this run shows.

**A5 positive control: PASS.** respawn restored every term_rsp and sigkill_cm episode.

**A7 dose.** helix and the watchdog made 29 decisions each, so the restoration gap is not a dose artefact. respawn made 233 decisions.

### Median blackout per scenario (s)

```
scenario        helix    watchdog  respawn  helix_v3params
crashloop_rsp   23.318   22.926    0.053    22.668
degrade_rsp      0.322    0.147    0.146     0.325
flaky_rsp        0.044    0.548    0.044     2.092
hang_camera      0.292    0.629   24.995     0.890
sigkill_cm       0.183    0.201    0.255     1.044
term_rsp         0.233    0.232    0.235     0.775
```

### Negatives and things that go against helix

- **crashloop_rsp.** robot_state_publisher has no backup. helix and the watchdog both
  declare a crash loop and go to SAFE_STOP, which leaves about 23 s of /tf blackout.
  - Plain respawn keeps /tf up by restarting it 72 to 74 times per episode.
  - When a node crashes periodically but every restart works for a while, blind respawn
    gives better availability than any policy that gives up.
  - HELIX gives up by design. A per-node "never give up" option is the obvious fix, but it is not built.
- **degrade_rsp.** On blackout, helix is worse (0.322 vs 0.147 s), because it restarts the degraded node and the restart causes a gap.
  - The watchdog never repairs the rate (FFF), so its short blackout comes with a topic that stays degraded for the rest of the episode.
  - This trade is real: a repair costs about 0.3 s of silence.
- **Diagnosis.** helix_v3params scored 18/18 against helix's 17/18. The adaptive envelope did not improve diagnosis here. Its gains are in repair latency and blackout (flaky_rsp 0.044 vs 2.092, sigkill_cm 0.183 vs 1.044).
- **helix_v3params** made 3 unnecessary interventions and 6 futile repairs. helix made 0 of each.

## Developmental set (13 scenarios, tuned on, not confirmatory)

```
policy            n  restored  safe  failed  diag_ok  decisions  unneeded  outage_mean_s  blackout_med_s
watchdog          39        33     0       6   24/33          47        6           2.82           0.593
helix             39        39     0       0   30/33          44        0           0.47           0.268
```

- The watchdog failed degrade_camera and degrade_cm, which have no rate rule.
- The watchdog made 3 unnecessary interventions each in deactivate_jsb and hang_cm.
- Paired blackout on 33 blocks: mean -0.144 s, 95% CI [-0.232, -0.050], Monte Carlo p = 0.006.
- These numbers come from the set the healer was tuned on.

## After the freeze (engineering, not confirmatory)

The following were added after the held-out run, in `4764f52` and later.

### Common-mode hold

When several unrelated nodes go stale within 0.5 s, the likely cause is shared, such as DDS, the host or the clock. In that case the healer holds for 2 s instead of restarting all of them.
- Covered by 4 gtests.
- Not exercised by any scenario: the post-fix developmental regression fired no hold events.

### Post-fix regression

The developmental set was run again with helix only, 1 rep, 13 episodes:
- 13/13 restored, 0 unnecessary interventions, diagnosis 10/11;
- outage mean 0.52 s, median blackout 0.290 s;
- CPU 1.4%, RSS 16.9 MB.

### Attach mode with `ros2 launch`

`eval/heal/humanoid/attach_test.py` covers attach mode. The same stack is started by a real launch file, and the healer adopts the running nodes by command-line pattern. Two crashes are injected on the real PIDs: SIGSEGV to robot_state_publisher at +8 s and SIGSEGV to controller_manager at +18 s. After the crashes, launch is sent SIGINT.

```
mode        rsp crash: who repaired   verified  /tf gap   cm crash: who repaired        verified  gap     orphans  survivors
respawn     launch, healer re-adopted   +1.6 s   1.128 s   launch, healer ran spawner      +0.49 s  0.498 s  0        0
norespawn   healer                      +1.6 s   1.172 s   healer, then spawner            +0.49 s  0.50 s   0        0
```

- In respawn mode, launch and the healer never ran a second copy of a node: there was 1 copy per node at the end.
- Two bugs were found and fixed on the way:
  - After launch exits, the healer releases the nodes it adopted. It stops any replacement it started itself.
  - The healer never adopts its own child. Before this fix, it re-adopted its own replacement while stopping it.
- The /tf gap of about 1.1 s after an rsp crash is the same whether launch or the healer restarts the node. It is the startup and discovery time of robot_state_publisher under this launch configuration, not healer overhead. The same restart took about 0.23 s in the spawned-mode study.
- **Remaining wart:** at launch shutdown, nodes exit slightly before launch does, and the healer restarts them for about 50 ms. When launch exits, the healer releases and stops them. No process survives this.
- **Limit:** a process the healer adopted is not its child, so its exit status is unknown. Crash and clean exit look the same.

### Public humanoid bag replay (false alarms)

The bag is one member of the BeyondMimic Unitree G1 rosbag set (Zenodo record 17529093, `rosbag_agile_motion`), 6.5 s long.
- It was copied with `eval/heal/humanoid/strip_bag_qos.py`, because Humble's player cannot parse the QoS metadata a newer distro wrote.
- It was replayed with `ros2 bag play --loop` as the only supervised node, using the spec `eval/heal/humanoid/bag_replay.spec`.
- Topics: /joint_states 500 Hz, /odom 500 Hz, /tf 1000 Hz.
- Nothing was injected, so every decision is a false alarm.

Result over 120 s: **1 unnecessary restart**. It happened at 6.78 s, at the first loop seam, while the envelope had about 3.5 s of data. The restart was verified 0.38 s later.
- The next 17 or so seams over 113 s raised no alarm.
- No process was left over.
- Running this exposed a healer bug: the healer appended `--ros-args -r __node:=<name>` to every exec line, and `ros2 bag play` reads `-r` as `--rate`. It now skips that for `ros2` CLI verbs other than `run`.

## Reproduction on the pinned freeze build

The held-out set was run a second time, on a clean checkout of the freeze commit `9e66b17`
(a separate worktree, `git status` empty), 3 reps. That commit predates the
`helix_v3params` arm, so the rerun has 4 arms and 108 episodes. Every episode exited rc=0.
Rows are in `heldout_humanoid_repro.csv`. The build predates automatic build stamping
(`eval/heal/provenance.py`), so the commit and the clean tree were checked by hand.

```
policy            n  restored  safe  failed  diag_ok  decisions  unneeded  outage_mean_s  blackout_med_s
none              27         9     0      18    0/18           0        0          24.68          24.985
respawn           27        21     0       6    9/18         220        0           8.31           0.157
watchdog          27        21     3       3   12/18          29        0           8.73           0.389
helix             27        24     3       0   16/18          29        0           4.43           0.274
```

- The restored / SAFE_STOP / failed grid is identical to the original run in every cell.
- P1 reproduces exactly: diff +3, p = 0.5, 0 vs 0 unnecessary interventions.
- **P2 reproduces in direction but not at p < 0.05.** Median -0.002 s, mean -0.132 s,
  95% bootstrap CI [-0.260, -0.009], exact sign-flip p = 0.0728, helix shorter in 8 of 15.
  The CI still excludes zero, so the kill criterion is still not met. The original
  p = 0.0477 was not robust to rerun noise, and it was already not significant after Holm.
- Diagnosis was 16/18 instead of 17/18. Both misses are term_rsp, labelled `hang` and
  `endpoint_loss`: a clean SIGTERM exit can be seen as silence before the process is reaped.
  The original run missed one term_rsp the same way. This label is timing-sensitive.

## What this does not show

- No robot: every number is software only, on one host.
- The held-out claims survive their kill criteria but are not significant after Holm. In the
  pinned rerun, P2 was p = 0.0728.
- The common-mode hold is verified by unit tests only.
- The attach test and the bag replay are single runs.
