# HELIX Demo Plan: The Post-Worthy Closed-Loop Video

Status: planned, not started. Owner: Yusuf. Target window: ~August 2026 (next
CaresLab session; HW work parked until then per the Summer-2026 sim-first plan).

## Why this doc exists

The current published demo (`youtu.be/PbKXB91-NSY`) is a runtime dashboard
replayed from the Session 8 bag. A PhD-level review (robotics / dependable
autonomy) and a self-review both reached the same verdict: it is a well-built
**scaffold**, but the headline demo is not impressive enough to post as a flex,
for two concrete reasons:

1. **Noise, not faults.** The Session 8 "30 anomalies" were statistical-noise
   crossings on an *idle* robot, not injected ground-truth faults. Our own
   `RESULTS.md` shows the Z-score detector has poor TPR on heavy-tailed spikes
   and that the threshold was raised to suppress idle jitter.
2. **Unactuated.** `/helix/cmd_vel` had 0 subscribers, so the 3,064 zero-twist
   "hold" messages had no physical effect. The robot never actually stopped.

So the pipeline is *proven as logic*, but the demo shows no real-world impact.
The fix is small and concrete, and it closes our standing P0.

## The one experiment that makes HELIX postable

**Film the robot physically stopping itself in response to a real injected
fault, while it is moving.** This converts "I built a pipeline that processes
events" into "the robot detected its own sensor failing and stopped itself."
Real footage, not a dashboard replay. This is also the standing P0 (twist_mux
wiring) and turns the "theater" critique into evidence.

### What must be true before filming

- [ ] `/helix/cmd_vel` wired to a real `twist_mux` fallback so a STOP_AND_HOLD
      actually preempts motion. (This is the P0. Sim verification harness for
      the priority mux already exists: `src/helix_bringup` twist_mux tests.)
- [ ] Robot is **walking** (teleop or a scripted forward trot) when the fault
      is injected, so the stop is visible.
- [ ] Fault is a **real injected fault**, not idle noise. Use the verified
      rate-drop method: `iptables -I INPUT -i enP8p1s0 -p udp -j DROP` on the
      Jetson to drop `/utlidar/cloud`. (Physical LiDAR cover does NOT reduce
      publish rate, confirmed Session 8: Livox keeps publishing.)
- [ ] Recovery allowlist constrained to `{STOP_AND_HOLD, LOG_ONLY}` for the
      first live moving run. No RESUME on first take.
- [ ] Safety: verify `body_height` (~0.32 m standing) before any pose command;
      operator hand on e-stop. (See the Session 5 damp-collapse incident.)
- [ ] NEVER call `motion_switcher` SelectMode with anything but `mcf`/`ai`.
      "normal" wedges the robot and needs a power cycle.

### What to film (shot list)

1. **Establishing shot:** GO2 walking normally, a terminal/overlay showing the
   anomaly detector healthy (z-scores low, rate nominal).
2. **The injection:** trigger the iptables rate-drop. On-screen marker so the
   viewer sees the moment.
3. **The reaction (the money shot):** anomaly fires -> diagnosis emits
   STOP_AND_HOLD -> `/helix/cmd_vel` goes zero-twist -> **robot stops walking.**
   Get this in one continuous take, robot + screen in frame together if
   possible (split-screen in post is fine).
4. **The explain tier (optional):** the local LLM annotation of what happened,
   shown as operator-facing text. Advisory only, off the safety path.
5. **B-roll:** the C++ detector resource overlay on the Jetson (low RAM/CPU).

### Numbers to capture for the post (real, during active motion)

- Detection latency: fault injected -> ANOMALY fired (target sub-second-ish).
- Decision -> actuation latency: ANOMALY -> robot velocity hits zero.
- True-positive on the injected fault (it must fire), and ideally a clean idle
  baseline showing it does NOT false-fire at the tuned threshold.
- C++ detector RAM/CPU on Jetson during the run.

## Honest framing rules for the eventual post

Keep the framing that a hostile reviewer endorsed. Do NOT drift toward stronger
claims. Specifically:

- Say "the robot detected a degraded sensor and stopped itself" ONLY once the
  actuation loop is closed and filmed. Until then it stays "issued, not
  actuated."
- Lead with engineering discipline (pure-function safety envelope, deterministic
  auditable rules, advisory-only LLM off the safety path, C++ hot path), not a
  "self-healing AI" hook.
- Raw event counts from a noisy idle session do not belong in post body text;
  show numbers in context (in the video) or use the injected-fault numbers above.

### Claims that make a reviewer cringe (banned)

- "Autonomous self-healing deployed on a GO2, thousands of safe stop commands."
- "Detected 30 critical anomalies and recovered the robot from danger."
- "AI that diagnoses root causes in real time and ensures safe operation."

## Decision

Do not post a HELIX video now. Post when the robot physically holds on a real
injected fault during motion. That is an ~August post, and it is the version
worth attaching a name to.

(If public momentum is wanted before then, HELIX is not the pick and a video is
not the medium; a written teardown of a different project is the better play.)
