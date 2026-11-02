"""Score fault-injection episodes written by run_episodes.py.

Outcome (from the independent probe, never from the healer's own claims):
  RESTORED  every spec output topic runs at >= 0.8x its nominal rate over the
            whole last 3 s of the episode
  SAFE      not restored, but the healer published SAFE_STOP on /helix/mode
  FAILED    neither
Nominal rate of a topic = the lowest declared rate among its providers, so a
working fallback counts as restored.

Decisions = healer events of kind act or given_up (what the healer chose to do).
  unnecessary  a decision on a node other than the injected root or its fallback
               (every decision in the nominal scenario is unnecessary)
  futile       verify_failed on the root (a repair was tried and did not work)
  diagnosis    the label of the last root decision is in the scenario's expect set
  left_burning the root still uses >= 1 core or >= 200 MB above its start at the
               end (a resource fault the outputs did not show but is still running)

  python3 eval/heal/score.py eval/heal/out/dev [--csv out.csv]
"""

import argparse
import csv
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_episodes import spec_outputs  # noqa: E402

OK_FRAC, WIN, TAIL, STEP = 0.8, 1.0, 3.0, 0.05


def nominal_rates(spec_text: str) -> dict[str, float]:
    rates: dict[str, float] = {}
    for outs in spec_outputs(spec_text).values():
        for topic, hz in outs:
            rates[topic] = min(rates.get(topic, hz), hz)
    return rates


def fallbacks(spec_text: str) -> dict[str, str]:
    fb = {}
    for line in spec_text.splitlines():
        tok = line.split("#", 1)[0].split()
        if len(tok) >= 2 and tok[0] == "node":
            for kv in tok[2:]:
                if kv.startswith("fallback="):
                    fb[tok[1]] = kv.split("=", 1)[1]
    return fb


def ok_series(arrivals: dict[str, list[float]], nominal: dict[str, float], t0: float, t1: float):
    """(t, ok) on a STEP grid: every topic has >= OK_FRAC * nominal msgs in the trailing WIN."""
    import bisect
    srt = {k: sorted(v) for k, v in arrivals.items()}
    out = []
    t = t0
    while t <= t1 + 1e-9:
        ok = True
        for topic, hz in nominal.items():
            a = srt.get(topic, [])
            n = bisect.bisect_right(a, t) - bisect.bisect_right(a, t - WIN)
            if n < OK_FRAC * hz * WIN:
                ok = False
                break
        out.append((t, ok))
        t += STEP
    return out


def score_episode(d: Path) -> dict | None:
    try:
        meta = json.loads((d / "meta.json").read_text())
        probe = json.loads((d / "probe.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    spec = (d / "spec").read_text()
    nominal = nominal_rates(spec)
    fb = fallbacks(spec)
    events = []
    t0_wall = None
    if (d / "events.jsonl").exists():
        for i, line in enumerate((d / "events.jsonl").read_text().splitlines()):
            e = json.loads(line)
            if i == 0 and "t0_wall" in e:
                t0_wall = e["t0_wall"]
                continue
            events.append(e)
    t_inj = probe["t_inject"] or meta["t_start"] + 7.0
    t_end = probe["t_end"]
    series = ok_series(probe["arrivals"], nominal, t_inj - 2.0, t_end)
    tail = [ok for t, ok in series if t >= t_end - TAIL]
    restored = bool(tail) and all(tail)
    safe = any("SAFE_STOP" in m for _, m in probe["modes"])
    outcome = "RESTORED" if restored else ("SAFE" if safe else "FAILED")
    post = [(t, ok) for t, ok in series if t >= t_inj]
    # Recovery latency: time from injection until ok holds for good (None if never).
    latency = None
    if restored:
        last_bad = max((t for t, ok in post if not ok), default=None)
        latency = 0.0 if last_bad is None else last_bad + STEP - t_inj
    outage = sum(STEP for _, ok in post if not ok)
    pre_ok = all(ok for t, ok in series if t < t_inj)

    root = meta["root"]
    roots = {root, fb.get(root, "")} - {""}
    decisions = [e for e in events if e["kind"] in ("act", "given_up")]
    unnecessary = [e for e in decisions if e["node"] not in roots]
    root_dec = [e for e in decisions if e["node"] == root]
    futile = sum(1 for e in events if e["kind"] == "verify_failed" and e["node"] == root)
    diag = root_dec[-1]["fault"] if root_dec else ""
    first_root_t = min((e["t"] for e in root_dec), default=None)
    # Event t is healer-relative; convert with t0_wall when present.
    detect_latency = None
    if first_root_t is not None and t0_wall is not None:
        detect_latency = t0_wall + first_root_t - t_inj
    left = False
    if (d / "snap.jsonl").exists() and root:
        lines = (d / "snap.jsonl").read_text().splitlines()
        first = {n["name"]: n for n in json.loads(lines[0])["nodes"]} if lines else {}
        last = {n["name"]: n for n in json.loads(lines[-1])["nodes"]} if lines else {}
        n = last.get(root)
        if n and n.get("alive", True):
            base = first.get(root, {}).get("rss", n["rss"])
            left = n["cpu"] >= 1.0 or n["rss"] - base >= 200
    return dict(
        scenario=meta["scenario"], policy=meta["policy"], rep=meta["rep"], root=root,
        restorable=meta["restorable"], outcome=outcome, pre_ok=pre_ok,
        latency_s=latency, outage_s=round(outage, 2), decisions=len(decisions),
        unnecessary=len(unnecessary), futile=futile, diagnosis=diag,
        diag_correct=(diag in meta["expect"]) if root else None,
        act_latency_s=detect_latency, inject_subs=probe["inject_subs"],
        healer_cpu=meta["healer_cpu_frac"], healer_rss_mb=meta["healer_rss_mb"],
        left_burning=left,
        actions=";".join(f"{e['node']}:{e['fault']}>{e['action']}" for e in decisions),
    )


def summarize(rows: list[dict]) -> str:
    by = defaultdict(list)
    for r in rows:
        by[r["policy"]].append(r)
    lines = ["policy    n  restored  safe  failed  diag_ok  unneeded  futile  burning  lat_med_s  outage_mean_s  cpu%  rss_mb"]
    for pol in ("none", "respawn", "watchdog", "helix"):
        rs = by.get(pol)
        if not rs:
            continue
        faulty = [r for r in rs if r["root"]]
        lat = [r["latency_s"] for r in faulty if r["latency_s"] is not None]
        diag = [r for r in faulty if r["diag_correct"]]
        n_out = {k: sum(1 for r in rs if r["outcome"] == k) for k in ("RESTORED", "SAFE", "FAILED")}
        lines.append(
            f"{pol:<9}{len(rs):>3}{n_out['RESTORED']:>10}{n_out['SAFE']:>6}{n_out['FAILED']:>8}"
            f"{len(diag):>5}/{len(faulty):<3}{sum(r['unnecessary'] for r in rs):>9}{sum(r['futile'] for r in rs):>8}{sum(r['left_burning'] for r in rs):>9}"
            f"{(statistics.median(lat) if lat else float('nan')):>11.2f}"
            f"{statistics.mean(r['outage_s'] for r in faulty) if faulty else float('nan'):>15.2f}"
            f"{100 * statistics.mean(r['healer_cpu'] for r in rs):>6.1f}{statistics.mean(r['healer_rss_mb'] for r in rs):>8.1f}")
    return "\n".join(lines)


def per_scenario(rows: list[dict]) -> str:
    cell = defaultdict(list)
    pols = []
    for r in rows:
        cell[(r["scenario"], r["policy"])].append(r)
        if r["policy"] not in pols:
            pols.append(r["policy"])
    scen = list(dict.fromkeys(r["scenario"] for r in rows))
    code = {"RESTORED": "R", "SAFE": "S", "FAILED": "F"}
    lines = ["scenario".ljust(26) + "".join(p.ljust(16) for p in pols)]
    for s in scen:
        parts = []
        for p in pols:
            rs = sorted(cell.get((s, p), []), key=lambda r: r["rep"])
            o = "".join(code[r["outcome"]] for r in rs)
            u = sum(r["unnecessary"] for r in rs)
            b = sum(r["left_burning"] for r in rs)
            parts.append((o + (f" u{u}" if u else "") + (f" b{b}" if b else "")).ljust(16))
        lines.append(s.ljust(26) + "".join(parts))
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    rows, missing = [], []
    for root in a.dirs:
        for d in sorted(p for p in Path(root).iterdir() if p.is_dir()):
            r = score_episode(d)
            (rows.append(r) if r else missing.append(d.name))
    bad = [r for r in rows if not r["pre_ok"] or (r["root"] and r["inject_subs"] == 0)]
    print(per_scenario(rows))
    print()
    print(summarize(rows))
    if missing:
        print(f"\nincomplete episodes: {missing}")
    for r in bad:
        print(f"WARNING {r['scenario']}/{r['policy']}/{r['rep']}: pre_ok={r['pre_ok']} inject_subs={r['inject_subs']}")
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
