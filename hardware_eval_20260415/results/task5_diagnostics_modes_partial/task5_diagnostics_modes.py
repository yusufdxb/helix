#!/usr/bin/env python3
"""Task 5 — Cycle GO2 modes, capture /diagnostics state per mode."""
import json, time, subprocess, os, re, sys

OUT = "/tmp/helix_t5"
os.makedirs(OUT, exist_ok=True)

def sh(cmd, timeout=20):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired as e:
        return (e.stdout.decode() if e.stdout else "") + (e.stderr.decode() if e.stderr else "") + "\n[TIMEOUT]"

def pub_request(topic, api_id, parameter=""):
    parameter_escaped = parameter.replace('"', '\\"')
    msg = (
        f'{{header: {{identity: {{id: 0, api_id: {api_id}}}, '
        f'lease: {{id: 0}}, policy: {{priority: 0, noreply: false}}}}, '
        f'parameter: "{parameter_escaped}", binary: []}}'
    )
    cmd = f"ros2 topic pub --once {topic} unitree_api/msg/Request '{msg}'"
    return sh(cmd, timeout=10)

def get_sport_mode():
    out = sh("ros2 topic echo --once /sportmodestate 2>&1", timeout=8)
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("mode:"):
            try:
                return int(s.split(":", 1)[1].strip())
            except Exception:
                return None
    return None

def topic_info_diag():
    out = sh("ros2 topic info -v /diagnostics 2>&1", timeout=10)
    pubs = []
    pub_count = 0
    sub_count = 0
    cur = {}
    in_pub = False
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("Publisher count:"):
            try: pub_count = int(s.split(":", 1)[1].strip())
            except: pass
        elif s.startswith("Subscription count:"):
            try: sub_count = int(s.split(":", 1)[1].strip())
            except: pass
        elif s == "Endpoint type: PUBLISHER":
            in_pub = True
            cur = {"endpoint_type": "PUBLISHER"}
        elif s == "Endpoint type: SUBSCRIPTION":
            if in_pub and cur:
                pubs.append(cur)
            in_pub = False
            cur = {}
        elif in_pub and s.startswith("Node name:"):
            cur["node_name"] = s.split(":", 1)[1].strip()
        elif in_pub and s.startswith("Node namespace:"):
            cur["node_namespace"] = s.split(":", 1)[1].strip()
    if in_pub and cur:
        pubs.append(cur)
    return {"publisher_count": pub_count, "subscription_count": sub_count, "publishers": pubs, "raw": out}

def topic_hz_diag(duration=12):
    out = sh(f"timeout {duration} ros2 topic hz /diagnostics 2>&1", timeout=duration + 5)
    rate = None
    for line in out.splitlines():
        m = re.search(r"average rate:\s*([\d.]+)", line)
        if m:
            try: rate = float(m.group(1))
            except: pass
    return {"rate_hz": rate, "raw_tail": "\n".join(out.splitlines()[-6:])}

def capture(label):
    ts_start = int(time.time())
    info = topic_info_diag()
    hz = topic_hz_diag(duration=12)
    sm = get_sport_mode()
    ts_end = int(time.time())
    rec = {
        "mode": label,
        "ts_start": ts_start,
        "ts_end": ts_end,
        "sportmodestate_mode": sm,
        "diagnostics": {
            "publisher_count": info["publisher_count"],
            "subscription_count": info["subscription_count"],
            "publishers": info["publishers"],
            "rate_hz": hz["rate_hz"],
            "hz_raw_tail": hz["raw_tail"],
        },
    }
    with open(f"{OUT}/diag_{label}.txt", "w") as f:
        f.write("--- topic info ---\n")
        f.write(info["raw"])
        f.write("\n--- topic hz tail ---\n")
        f.write(hz["raw_tail"])
        f.write(f"\n--- sportmodestate.mode = {sm}\n")
    return rec

def step(label, send_fn=None, dwell=30):
    print(f"[{time.strftime('%H:%M:%S')}] === STEP {label} ===", flush=True)
    if send_fn is not None:
        out = send_fn()
        with open(f"{OUT}/cmd_{label}.txt", "w") as f:
            f.write(out)
    if dwell:
        print(f"  dwell {dwell}s ...", flush=True)
        time.sleep(dwell)
    rec = capture(label)
    print(f"  captured: pubs={rec['diagnostics']['publisher_count']} "
          f"rate_hz={rec['diagnostics']['rate_hz']} sport_mode={rec['sportmodestate_mode']}", flush=True)
    return rec

records = []

# 0) Baseline — robot in current state, no command sent
records.append(step("baseline_idle", send_fn=None, dwell=2))

# 1) Damp — release torque, lay
records.append(step("damp", send_fn=lambda: pub_request("/api/sport/request", 1001), dwell=30))

# 2) Stand up
records.append(step("stand_up", send_fn=lambda: pub_request("/api/sport/request", 1004), dwell=30))

# 3) Balance stand — locomotion engaged, no translation
records.append(step("balance_stand_walk", send_fn=lambda: pub_request("/api/sport/request", 1002), dwell=30))

# 4) Obstacles avoid switch ON
records.append(step("obstacles_avoid_on",
                    send_fn=lambda: pub_request("/api/obstacles_avoid/request", 1001, parameter='{"enable":true}'),
                    dwell=30))

# 5) Motion switcher SELECT_MODE "ai" (high-level mode)
records.append(step("motion_switcher_ai",
                    send_fn=lambda: pub_request("/api/motion_switcher/request", 1002, parameter='{"name":"ai"}'),
                    dwell=30))

# 6) Cleanup — switch back to normal motion mode + obstacles_avoid OFF + Damp
print("[CLEANUP] motion_switcher -> normal", flush=True)
pub_request("/api/motion_switcher/request", 1002, parameter='{"name":"normal"}')
time.sleep(5)
print("[CLEANUP] obstacles_avoid -> off", flush=True)
pub_request("/api/obstacles_avoid/request", 1001, parameter='{"enable":false}')
time.sleep(2)
print("[CLEANUP] sport -> Damp", flush=True)
pub_request("/api/sport/request", 1001)
time.sleep(3)
final_mode = get_sport_mode()

summary = {
    "task": "Task 5 — /diagnostics mode dependency on GO2",
    "ts_start": records[0]["ts_start"] if records else None,
    "ts_end": int(time.time()),
    "modes_tested": [r["mode"] for r in records],
    "diagnostics_publisher_in_any_mode": any(r["diagnostics"]["publisher_count"] > 0 for r in records),
    "any_mode_with_rate": any((r["diagnostics"]["rate_hz"] or 0) > 0 for r in records),
    "final_sport_mode_after_cleanup": final_mode,
    "records": records,
}
with open(f"{OUT}/diagnostics_mode_matrix.json", "w") as f:
    json.dump(summary, f, indent=2)
print(json.dumps({k: v for k, v in summary.items() if k != "records"}, indent=2))
print(f"DONE: {OUT}")
