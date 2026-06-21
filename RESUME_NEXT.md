# HELIX: Next Session Resume Card

**Trigger phrase:** "implement helix next steps"
**Working dir:** depends on which PC. T7 path is `/media/<cares|careslab>/T7 Storage/LABWORK/HELIX/`
**Session log target:** `hardware_eval_<YYYYMMDD>/notes/SESSION_NOTES.md`

## State going in (updated 2026-05-09, post-Session 8)

**Session 8 (2026-04-23) closed the loop on live hardware.** Full
SENSE -> DIAGNOSE -> RECOVER ran on Jetson against the GO2: 30 ANOMALY
faults -> 14 RecoveryHints (9x R1 STOP_AND_HOLD on utlidar, 5x R2
RESUME) -> 14 audited actions (7 ACCEPTED, 7 SUPPRESSED_COOLDOWN) ->
3064 zero-twist `/helix/cmd_vel` messages during STOP. The required R1
schema fix has since landed on `main` (`82f7a15`). The min-duration
anomaly gate (`15c2e25`) was added afterward to suppress the 9
transient idle STOP_AND_HOLDs/min observed during Session 8.

The next P0 is **booking a CaresLab session** for `twist_mux` fallback
wiring + a post-gate idle-FP re-measurement on live hardware.

### Quick status

| Item | State | Evidence |
|------|-------|----------|
| Closed-loop on live GO2 | DONE (S8) | `hardware_eval_20260423/results/closed_loop_demo.md` |
| R1 schema fix on main | LANDED | `82f7a15` |
| Min-duration anomaly gate (Py + C++) | LANDED, NOT HW-MEASURED | `15c2e25` |
| Stale-topic NaN -> ANOMALY (Py + C++) | LANDED | `09531d3`, `0409236` |
| C++ AnomalyDetector 30-min parity | RUN | RSS 19.81 MB (-56%), CPU 0.80% (-60%); see `cpp_parity_summary.md` |
| C++ promoted to default | BLOCKED | RSS 44% missed 30% design target; `use_cpp_anomaly:=true` stays off |
| `/helix/cmd_vel` wired to actuation | NOT WIRED | 0 downstream subs, STOP_AND_HOLD is a void publish |
| Post-gate idle FP re-measurement | NOT RUN | needs lab session |
| Task 4 (8-hr stability), Task 7 (rosout ring) | NOT STARTED, P2 | (none) |

### Open caveats (from `docs/LIMITATIONS.md`)

- **R1 schema-fix caveat (now stale-but-untouched in doc).** README
  Reproducibility note + LIMITATIONS still cite the
  `fix/r1-anomaly-schema-mismatch` branch. As of `82f7a15` that fix is
  on `main`. Update both before promoting the demo.
- **Recovery-wiring caveat.** `helix_recovery` is a control-plane
  component, NOT a physical safety intervention until `/helix/cmd_vel`
  is on a `twist_mux` fallback.
- **Stale-topic NaN gap (closed in code, not re-measured).** PR #7/#8
  added NaN-as-stale-anomaly. Session 8's UDP-block capture predates
  the fix; a fresh injection should now fire ANOMALYs during the drop.

### Untracked items (Yusuf's call, do not touch)

- `hardware_eval_20260414/` and `hardware_eval_20260415/` are still
  untracked locally. Pending T7 mirror verification before commit.

## Pre-flight for next session (block on failures)

```bash
mountpoint -q "/media/cares/T7 Storage" || mountpoint -q "/media/careslab/T7 Storage"
ping -c1 -W2 192.168.0.2
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.0.2 \
  "ping -c1 -W2 192.168.123.161"
sshpass -p 123 ssh unitree@192.168.0.2 \
  "source /opt/ros/humble/setup.bash; source ~/helix_ws/install/setup.bash; \
   ros2 interface show helix_msgs/msg/FaultEvent | head -3"
```

## Suggested P0/P1 for next session

1. **(P0) Wire `/helix/cmd_vel` -> `twist_mux` fallback** so STOP_AND_HOLD
   actually arrests motion. Gating item for honest "physical safety"
   framing.
2. **(P0) Re-measure idle FP rate with `min_anomaly_duration_s=2.0`.**
   Pre-gate baseline was 9 STOP_AND_HOLDs in 7 min idle.
3. **(P1) Update README + LIMITATIONS** to drop the
   `fix/r1-anomaly-schema-mismatch` branch reference.
4. **(P1) Re-run UDP-block injection** for stale-topic NaN path.
5. **(P2) Task 4 (8-hr stability), Task 7 (rosout ring buffer).**

## Hard rules (carry-forward, unchanged)

- Do NOT claim hardware validation unless it actually ran this session.
- Every claim cites an artifact under `hardware_eval_<DATE>/results/`.
- Raw bags stay on T7; commit only JSON/txt/md summaries.
- No `Co-Authored-By: Claude` in commits.
- For sport commands that release torque/pose: verify `body_height`
  from `sportmodestate` first (see `incident_damp_collapse.md`).

## Reference

- Session 8: `hardware_eval_20260423/results/closed_loop_demo.md`,
  `cpp_parity_summary.md`
- Limitations: `docs/LIMITATIONS.md`
- Status / tasks: `obsidian-vault/Projects/HELIX/`
