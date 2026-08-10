#!/usr/bin/env python3
"""Build the HELIX Runtime Console from recorded bag telemetry.

Inputs (assets/):
  events.json  extracted from bags/post_fix_demo (30 faults, 14 hints,
               14 actions, 3064 cmd_vel) by helix-demo-build/extract_events.py
  rate.json    per-second publish rate of /sportmodestate over the same bag
  *.woff2      subset display + data faces, inlined so the page is standalone

Outputs:
  helix_console.html           standalone page, open directly in a browser
  helix_console.artifact.html  body-only fragment for hosted publishing

Every number rendered by the console comes from these two files. No value in
the page is hand-written, so a re-extract from a new bag re-renders truthfully.
"""

import base64
import json
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
ASSETS = HERE / "assets"

# The recovery envelope's cooldown window; mirrors the `cooldown_seconds`
# default declared in helix_recovery/recovery_node.py.
COOLDOWN_S = 5.0

# A gap longer than this between consecutive /helix/cmd_vel messages ends a
# hold window. The publisher runs at 20 Hz, so 1 s is ~20 missed messages.
HOLD_GAP_S = 1.0

# The detector threshold in force when this bag was recorded. The smallest |z|
# in the bag is 3.01, so the run used zscore_threshold 3.0; the bump to 4.0
# (b9b3a71, PR #6) and the min-duration gate (15c2e25) both landed afterwards.
# build_run() asserts this against the data rather than trusting the constant.
BAG_THRESHOLD = 3.0


def context(fault, key):
    """Pull a value out of a FaultEvent's parallel context key/value arrays."""
    try:
        return fault["cvals"][fault["ckeys"].index(key)]
    except (ValueError, IndexError):
        return None


def hold_windows(times):
    """Collapse cmd_vel timestamps into contiguous [start, end] hold windows."""
    times = sorted(times)
    if not times:
        return []
    windows, start, prev = [], times[0], times[0]
    for t in times[1:]:
        if t - prev > HOLD_GAP_S:
            windows.append([round(start, 2), round(prev, 2)])
            start = t
        prev = t
    windows.append([round(start, 2), round(prev, 2)])
    return windows


def build_run():
    raw = json.loads((ASSETS / "events.json").read_text())
    rate = json.loads((ASSETS / "rate.json").read_text())
    events = raw["events"]

    faults = [
        {
            "topic": "fault",
            "t": e["t"],
            "metric": e["node"],
            "ftype": e["ftype"],
            "severity": e["severity"],
            "z": float(context(e, "zscore") or 0.0),
            "current": context(e, "current_value"),
            "mean": context(e, "window_mean"),
            "std": context(e, "window_std"),
        }
        for e in events
        if e["topic"] == "fault"
    ]
    hints = [
        {
            "topic": "hint",
            "t": e["t"],
            "rule": e["rule"],
            "action": e["action"],
            "confidence": e["confidence"],
            "reasoning": e["reasoning"],
            "fault_id": e["fault_id"],
        }
        for e in events
        if e["topic"] == "hint"
    ]
    actions = [
        {
            "topic": "action",
            "t": e["t"],
            "action": e["action"],
            "status": e["status"],
            "fault_id": e["fault_id"],
        }
        for e in events
        if e["topic"] == "action"
    ]
    holds = hold_windows([e["t"] for e in events if e["topic"] == "cmd_vel"])

    z_min = min(abs(f["z"]) for f in faults)
    if not BAG_THRESHOLD <= z_min < BAG_THRESHOLD + 1.0:
        raise SystemExit(
            f"threshold mismatch: smallest |z| in the bag is {z_min}, which does "
            f"not sit just above the declared BAG_THRESHOLD {BAG_THRESHOLD}"
        )

    # The console replays one merged stream; faults must land before the hint
    # they produced when timestamps tie, so sort by (time, tier).
    tier = {"fault": 0, "hint": 1, "action": 2}
    merged = sorted(faults + hints + actions, key=lambda e: (e["t"], tier[e["topic"]]))

    return {
        "duration": raw["summary"]["bag_duration"],
        "bin": rate["bin"],
        "rate": rate["rate"],
        "cooldown": COOLDOWN_S,
        "threshold": BAG_THRESHOLD,
        "z_min": round(z_min, 2),
        "holds": holds,
        "faults": faults,
        "actions": actions,
        "events": merged,
        "counts": {
            "faults": len(faults),
            "hints": len(hints),
            "actions": len(actions),
            "cmd_vel": raw["summary"]["n_cmd_vel"],
        },
    }


def b64(name):
    return base64.b64encode((ASSETS / name).read_bytes()).decode()


def main():
    run = build_run()
    page = (HERE / "console.template.html").read_text()
    page = page.replace("__FONT_LMCOND__", b64("lmcond.woff2"))
    page = page.replace("__FONT_NOTO_R__", b64("noto-r.woff2"))
    page = page.replace("__FONT_NOTO_B__", b64("noto-b.woff2"))
    page = page.replace("__DATA__", json.dumps(run, separators=(",", ":")))

    (HERE / "helix_console.artifact.html").write_text(page)
    (HERE / "helix_console.html").write_text(
        '<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width,initial-scale=1">\n'
        f"</head>\n<body>\n{page}\n</body>\n</html>\n"
    )

    c = run["counts"]
    print(f"events   {len(run['events'])} merged "
          f"({c['faults']} fault / {c['hints']} hint / {c['actions']} action)")
    print(f"cmd_vel  {c['cmd_vel']} msgs in {len(run['holds'])} hold windows: {run['holds']}")
    print(f"rate     {len(run['rate'])} bins over {run['duration']} s")
    for out in ("helix_console.html", "helix_console.artifact.html"):
        print(f"wrote    dashboard/{out}  {(HERE / out).stat().st_size / 1024:.0f} KB")


if __name__ == "__main__":
    main()
