# HELIX Runtime Console

A single-file console that replays a recorded HELIX run and shows the recovery
plane working: a fault enters at SENSE, propagates through CONTEXT and
DIAGNOSE, and leaves RECOVER as a zero-twist command stream.

Open `helix_console.html` in any browser. No server, no build step, no network:
fonts and telemetry are inlined.

```
python3 dashboard/build_console.py    # regenerate both outputs
```

## What is on screen

| Panel | Source |
|---|---|
| Tier strip | live replay state of the four lifecycle tiers |
| Signal | `/sportmodestate` publish rate, 1 s bins, hold windows shaded |
| Detector channels | the 30 ANOMALY faults, one row per `rate_hz/*` metric, tick height proportional to \|z\| |
| Event console | the merged fault / hint / action stream at replay time |
| Recovery envelope | envelope state, live cooldown remaining, action allowlist |
| Transport | the whole 439 s run: fault ticks, HOLD windows, suppressed RESUMEs |

## Data provenance

Everything comes from `bags/post_fix_demo` recorded on a live Unitree GO2 EDU
at CaresLab on 2026-04-23 (Session 8), with the four-tier stack running on the
onboard Jetson Orin NX:

- 30 ANOMALY faults, 14 recovery hints, 14 recovery actions, 3,064 zero-twist
  `/helix/cmd_vel` messages
- two hold windows, `[10.22, 15.75]` and `[161.70, 309.25]` s, derived from the
  cmd_vel timestamps rather than asserted

`assets/events.json` and `assets/rate.json` are the extracted telemetry. No
number in the page is hand-written; re-extract from a different bag and the
console re-renders truthfully.

### Threshold check

The build asserts the declared detector threshold against the data. The
smallest \|z\| in this bag is 3.01, which places the run at
`zscore_threshold: 3.0`; the bump to 4.0 (`b9b3a71`, PR #6) and the
min-duration gate (`15c2e25`) both landed after this recording. The build
fails loudly if a future bag contradicts the constant.

## What the run honestly shows

The 147-second hold from t+161.7 is too long. Three RESUMEs were rejected as
`SUPPRESSED_COOLDOWN` because a RESUME was being checked against the cooldown
of the STOP it was trying to clear. This bag predates the fix; RESUME is now
exempt from cooldown (`052bd24`) with regression tests. The console shows the
bug rather than hiding it.

Two limits stated on the page itself: the robot stood idle for the whole run,
so all 30 anomalies are natural jitter and nine STOP_AND_HOLDs in seven minutes
is a false-positive rate still being tuned; and `/helix/cmd_vel` had no
subscriber, so the hold is proven through the software path but not yet through
twist_mux to the motors.

## Not yet built

A live mode. The same page can be driven from a websocket bridge instead of a
replay clock, which turns it into an operator view during a lab session. The
event schema the console consumes is deliberately close to the ROS messages.

## Files

```
build_console.py             builder: telemetry + fonts -> standalone page
console.template.html        markup, styles, replay engine
assets/events.json           extracted fault/hint/action/cmd_vel stream
assets/rate.json             per-second /sportmodestate publish rate
assets/*.woff2               subset display and data faces
helix_console.html           generated, standalone
helix_console.artifact.html  generated, body-only fragment for hosting
```
