# Incident — Damp on standing GO2 caused controlled collapse

**When:** 2026-04-15 ~23:05 UTC, during Task 5 (mode dependency).
**Outcome:** No damage. Robot recovered with `RecoveryStand` (api_id 1006).

## What happened

Task 5 orchestrator script (`task5_diagnostics_modes.py`) sent
`ROBOT_SPORT_API_ID_DAMP = 1001` to `/api/sport/request` as the second step,
expecting the robot to enter a benign "lay loose" state. At the time of the
command, the robot was actually **standing** (`body_height ≈ 0.32 m`,
`accelerometer Z ≈ 9.24 m/s²`, body level). Damp releases all motor torque,
so the standing robot collapsed. Operator (Yusuf, physically present) reported
the fall. Script was killed; Damp was re-issued (now safely, robot already on
the floor); robot was then recovered using `RecoveryStand`.

## Root-cause analysis

`sportmodestate.mode = 0` was misread as "currently laying / idle physical
pose". In Unitree firmware, `mode = 0` means **no active sport command is
currently running**, NOT "robot is on the ground". The robot's *physical
pose* must be inferred from `body_height` (≈ 0.07 m when down,
≈ 0.32 m when standing) or from the IMU pose channel — not from the `mode`
field.

The pre-Damp baseline echo *did* contain the body_height clue, but the
script-level pose check only inspected `mode`. The robot had been standing
since boot (probably from the prior come-here / phoenix session), so issuing
Damp without a pose check dropped it.

## Concrete preventive rules (apply to all future Task-5/6/7 work)

1. **Before any sport command that releases torque or changes pose
   (`Damp`, `StandDown`, `Sit`), read `body_height` from `sportmodestate`.**
   If `body_height > 0.15 m`, treat the robot as standing. Either issue a
   gentle `StandDown` (api_id 1005) first, or skip Damp.
2. **Do not infer physical pose from `sportmodestate.mode`.** Mode is a
   command-state field, not a posture field.
3. **Take the robot to a known pose at the start of any mode-cycling
   experiment.** Either `RecoveryStand` (1006) → known standing, or `StandDown`
   (1005) → known low. Then proceed deterministically.
4. **The proper safe-shutdown sequence is: `StandDown` (1005) → wait until
   `body_height < 0.10 m` → `Damp` (1001).** Never go straight to Damp from
   standing.
5. **For remote-execution runs (operator at the robot but commands issued
   over SSH), pause for explicit human go-ahead between every pose-changing
   command** until a verified-safe sequence has been validated. Don't trust
   a long-running unattended script the way a Task-3-style passive monitor
   can be trusted.

## What this means for the diagnostics_mode_matrix.json record

The `damp_on_standing` row is recorded with the `incident` field. The
`/diagnostics` data (0 publishers) is still valid for that capture — the
topic graph is independent of the Damp mishap. No data was fabricated.
