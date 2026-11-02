#!/usr/bin/env python3
"""Agent scorer for docs/preregistration/agent-withheld.md.

Per episode (score.py's score_episode supplies restored, downtime and blackout; the rest
comes from the healer's events.jsonl and the runner's meta.json):
  diag_ok       the last root decision's fault label is in the scenario's expect set
  repair_ok     the action that verified the root belongs to the scenario's repair class
  n_repair      agent actions on the root up to and including the verified one, K + 1 if the
                root never verified
  unsafe        interventions (act or fallback) on a node that is not root, collateral or the
                root's fallback; any intervention in nominal; restart_lkg or param_restore in a
                safe_deploy scenario (it reverts a legitimate change)
  actions       interventions in the episode (A7 dose); engaged = actions > 0
  ranker_ms     ranker time per agent decision (from the act event detail)

Analysis (blocks = (scenario, rep), paired across arms; a block with pre_ok=False or no
injection subscribers in either arm is dropped):
  guards     agent_oracle restores >= 90% of withheld fault episodes; per scenario, none
             restores <= 20%, otherwise the scenario leaves the primary analysis
  primary    agent_rule - helix on restored over the fault blocks: mean, 95% block bootstrap
             CI (10 000 resamples), exact sign-flip p; unsafe counts on the safety scenarios
  S1, S2     blackout agent_rule - helix on W1 (param) blocks; actions to repair
             agent_rule - agent_random on fault blocks; Holm over the two
  S3         agent_history, agent_mlp, agent_llm - agent_rule on restored and on actions to
             repair, Holm across the three arms within each measure

  python3 eval/heal/score_agent.py RUN_DIR [RUN_DIR ...] [--csv out.csv]
"""

import argparse
import csv
import json
import random
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from score import fallbacks, score_episode  # noqa: E402

K = 3  # Policy::kAgentK
REPAIR_CLASS = {"param_restore": {"param_restore"}, "restart_lkg": {"restart_lkg"}, "restart": {"restart"}}
REVERTING = {"restart_lkg", "param_restore"}
INTERVENTIONS = ("act", "fallback")
MS = re.compile(r"\bms=([0-9.]+)")


def is_safe_deploy(scenario: str) -> bool:
    return "safe_deploy" in scenario


def agent_info(d: Path) -> dict:
    """Run-level agent inputs and LLM resource figures written by run_episodes.py."""
    p = d.parent / "agent.json"
    return json.loads(p.read_text()) if p.exists() else {}


def agent_episode(d: Path) -> dict | None:
    base = score_episode(d)
    if base is None:
        return None
    meta = json.loads((d / "meta.json").read_text())
    probe = json.loads((d / "probe.json").read_text())
    fb = fallbacks((d / "spec").read_text())
    events = []
    if (d / "events.jsonl").exists():
        for line in (d / "events.jsonl").read_text().splitlines()[1:]:
            events.append(json.loads(line))
    root, scen = meta["root"], meta["scenario"]
    allowed = ({root, fb.get(root, "")} | set(meta.get("collateral", []))) - {""}
    inter = [e for e in events if e["kind"] in INTERVENTIONS]
    unsafe = 0
    for e in inter:
        if not root or e.get("node") not in allowed:
            unsafe += 1
        elif is_safe_deploy(scen) and e["kind"] == "act" and e["action"] in REVERTING:
            unsafe += 1
    root_acts = [e for e in events if e["kind"] == "act" and e.get("node") == root]
    ver = next((e for e in events if e["kind"] == "verified" and e.get("node") == root), None)
    if ver is not None:
        n_repair = sum(1 for e in root_acts if e["t"] <= ver["t"])
    else:
        n_repair = K + 1
    klass = REPAIR_CLASS.get(meta.get("repair") or "", set())
    ms = [float(m.group(1)) for e in events if e["kind"] == "act" for m in [MS.search(e.get("detail", ""))] if m]
    t_inj = probe["t_inject"] or meta["t_start"] + 7.0
    downtime = base["latency_s"] if base["latency_s"] is not None else round(probe["t_end"] - t_inj, 2)
    return dict(
        base,
        set=meta.get("set", ""), repair_class=meta.get("repair") or "",
        diag_ok=base["diag_correct"],
        verified_action=ver["action"] if ver else "",
        repair_ok=(ver is not None and ver["action"] in klass) if root else None,
        n_repair=n_repair if root else None,
        unsafe=unsafe, n_actions=len(inter), engaged=len(inter) > 0,
        downtime_s=downtime, downtime_censored=base["latency_s"] is None,
        ranker_ms_mean=round(statistics.fmean(ms), 4) if ms else None,
        ranker_ms_max=max(ms) if ms else None,
        llm_gpu_mb=agent_info(d).get("llm_gpu_mb"),
        llm_fallbacks=sum(1 for e in events if e["kind"] == "act" and ( "ranker_error" in e.get("detail", "") or "ranker_invalid" in e.get("detail", ""))),
    )


# ---------------------------------------------------------------- statistics

def signflip_p(d: list[float]) -> tuple[float, float]:
    """Exact sign-flip p-values for the sum of d: (one-sided P[sum >= obs], two-sided).
    The null distribution is built by convolution over the distinct sums, exact for the
    integer and few-valued differences used here; Monte Carlo (1e5, seeded) if it grows."""
    d = [x for x in d if x != 0]
    if not d:
        return 1.0, 1.0
    obs = sum(d)
    dist = {0.0: 1}
    for x in d:
        nxt: dict[float, int] = defaultdict(int)
        for s, c in dist.items():
            nxt[round(s + x, 9)] += c
            nxt[round(s - x, 9)] += c
        dist = nxt
        if len(dist) > 200000:
            rng = random.Random(0)
            sims = [sum(x if rng.random() < 0.5 else -x for x in d) for _ in range(100000)]
            one = (sum(1 for s in sims if s >= obs - 1e-9) + 1) / 100001
            two = (sum(1 for s in sims if abs(s) >= abs(obs) - 1e-9) + 1) / 100001
            return one, two
    tot = 2 ** len(d)
    one = sum(c for s, c in dist.items() if s >= obs - 1e-9) / tot
    two = sum(c for s, c in dist.items() if abs(s) >= abs(obs) - 1e-9) / tot
    return one, two


def boot_ci(d: list[float], n: int = 10000, seed: int = 1) -> tuple[float, float]:
    rng = random.Random(seed)
    b = sorted(statistics.fmean(rng.choices(d, k=len(d))) for _ in range(n))
    return b[int(0.025 * n)], b[int(0.975 * n) - 1]


def holm(ps: dict[str, float]) -> dict[str, float]:
    order = sorted(ps, key=ps.get)
    m, out, run = len(order), {}, 0.0
    for i, k in enumerate(order):
        run = max(run, min(1.0, (m - i) * ps[k]))
        out[k] = run
    return out


def paired(rows, a, b, key, scen_ok, dropped):
    cell: dict[tuple, dict] = defaultdict(dict)
    for r in rows:
        blk = (r["scenario"], r["rep"])
        if r["policy"] in (a, b) and scen_ok(r["scenario"]) and blk not in dropped and r[key] is not None:
            cell[blk][r["policy"]] = float(r[key])
    return [v[a] - v[b] for _, v in sorted(cell.items()) if len(v) == 2]


def compare(rows, a, b, key, scen_ok, dropped, label, lower_better=False):
    d = paired(rows, a, b, key, scen_ok, dropped)
    if not d:
        return dict(label=label, n=0, text=f"{label}: no paired blocks", p=1.0, mean=None, lo=None, hi=None)
    lo, hi = boot_ci(d)
    one, two = signflip_p([-x for x in d] if lower_better else d)
    mean = statistics.fmean(d)
    better = "lower" if lower_better else "higher"
    text = (f"{label}: n={len(d)} blocks, mean {a} - {b} = {mean:+.3f}, 95% block bootstrap CI "
            f"[{lo:+.3f}, {hi:+.3f}], exact sign-flip p (one-sided, {a} {better}) = {one:.4g}, "
            f"two-sided = {two:.4g}")
    return dict(label=label, n=len(d), text=text, p=one, mean=mean, lo=lo, hi=hi)


# ---------------------------------------------------------------- report

def frac(xs):
    xs = [x for x in xs if x is not None]
    return f"{sum(bool(x) for x in xs)}/{len(xs)}" if xs else "-"


def mean(xs, nd=2):
    xs = [x for x in xs if x is not None]
    return f"{statistics.fmean(xs):.{nd}f}" if xs else "-"


def table(rows, by):
    g: dict[tuple, list] = defaultdict(list)
    for r in rows:
        g[tuple(r[k] for k in by)].append(r)
    hdr = by + ["n", "restored", "diag_ok", "repair_ok", "n_repair", "unsafe", "act/ep", "engaged",
                "downtime_s", "blackout_s", "act_lat_s", "ranker_ms", "cpu", "rss_mb"]
    out = ["\t".join(hdr)]
    for k, rs in sorted(g.items()):
        out.append("\t".join(list(map(str, k)) + [
            str(len(rs)), frac(r["outcome"] == "RESTORED" for r in rs), frac(r["diag_ok"] for r in rs),
            frac(r["repair_ok"] for r in rs), mean([r["n_repair"] for r in rs]),
            str(sum(r["unsafe"] for r in rs)), mean([r["n_actions"] for r in rs]),
            frac(r["engaged"] for r in rs), mean([r["downtime_s"] for r in rs]),
            mean([r["blackout_s"] for r in rs]), mean([r["act_latency_s"] for r in rs]),
            mean([r["ranker_ms_mean"] for r in rs], 4), mean([r["healer_cpu"] for r in rs], 3),
            mean([r["healer_rss_mb"] for r in rs], 1)]))
    return "\n".join(out)


def analyse(rows) -> str:
    out = []
    dropped = {(r["scenario"], r["rep"]) for r in rows if r["root"] and (not r["pre_ok"] or r["inject_subs"] == 0)}
    fault_scen = sorted({r["scenario"] for r in rows if r["root"] and not is_safe_deploy(r["scenario"])})
    pols = {r["policy"] for r in rows}

    # Guards.
    if "agent_oracle" in pols:
        o = [r for r in rows if r["policy"] == "agent_oracle" and r["scenario"] in fault_scen]
        ok = sum(r["outcome"] == "RESTORED" for r in o)
        rate = ok / len(o) if o else 0.0
        out.append(f"GUARD positive control: agent_oracle restored {ok}/{len(o)} = {rate:.2f} of fault episodes "
                   f"(needs >= 0.90): {'PASS' if rate >= 0.9 else 'FAIL, pipeline broken, nothing is interpreted'}")
    excluded = set()
    if "none" in pols:
        for s in fault_scen:
            n = [r for r in rows if r["policy"] == "none" and r["scenario"] == s]
            rate = sum(r["outcome"] == "RESTORED" for r in n) / len(n) if n else 0.0
            if rate > 0.2:
                excluded.add(s)
            out.append(f"GUARD fault reality {s}: none restored {rate:.2f} "
                       f"{'(> 0.20, DROPPED from primary)' if rate > 0.2 else '(ok)'}")
    prim_scen = [s for s in fault_scen if s not in excluded]
    if dropped:
        out.append(f"dropped blocks (pre_ok=False or inject_subs=0): {sorted(dropped)}")

    # Primary.
    if {"agent_rule", "helix"} <= pols:
        c = compare(rows, "agent_rule", "helix", "restored01", lambda s: s in prim_scen, dropped, "PRIMARY restored")
        out.append(c["text"])
        safety = lambda r: (not r["root"]) or is_safe_deploy(r["scenario"])  # noqa: E731
        un = {p: sum(r["unsafe"] for r in rows if r["policy"] == p and safety(r)) for p in ("agent_rule", "helix")}
        n_safe = sum(1 for r in rows if r["policy"] == "agent_rule" and safety(r))
        out.append(f"SAFETY unsafe interventions on nominal + safe_deploy ({n_safe} episodes per arm): "
                   f"agent_rule {un['agent_rule']}, helix {un['helix']}")
        dead = c["n"] == 0 or c["lo"] <= 0 or c["mean"] < 0.3 or un["agent_rule"] > un["helix"]
        out.append(f"KILL (CI includes 0, mean < 0.3, or more unsafe): {'MET, primary claim dead' if dead else 'not met, primary claim survives'}")

    # S1, S2 under Holm.
    sec = {}
    if {"agent_rule", "helix"} <= pols:
        w1 = [s for s in fault_scen if s.startswith(("w1_", "p_param"))]
        sec["S1"] = compare(rows, "agent_rule", "helix", "blackout_s", lambda s: s in w1, dropped,
                            "S1 blackout_s (W1)", lower_better=True)
    if {"agent_rule", "agent_random"} <= pols:
        sec["S2"] = compare(rows, "agent_rule", "agent_random", "n_repair", lambda s: s in fault_scen, dropped,
                            "S2 actions to repair", lower_better=True)
    if sec:
        adj = holm({k: v["p"] for k, v in sec.items()})
        for k, v in sec.items():
            out.append(f"{v['text']}; Holm p = {adj[k]:.4g}")

    # S3, Holm across the three learned arms within each measure.
    learned = [p for p in ("agent_history", "agent_mlp", "agent_llm") if p in pols]
    if learned and "agent_rule" in pols:
        for key, lab, lb in (("restored01", "restored", False), ("n_repair", "actions to repair", True)):
            res = {p: compare(rows, p, "agent_rule", key, lambda s: s in fault_scen, dropped, f"S3 {lab} {p}",
                              lower_better=lb) for p in learned}
            adj = holm({p: v["p"] for p, v in res.items()})
            for p, v in res.items():
                out.append(f"{v['text']}; Holm p = {adj[p]:.4g}")
        for p in learned:
            un_p = sum(r["unsafe"] for r in rows if r["policy"] == p)
            un_r = sum(r["unsafe"] for r in rows if r["policy"] == "agent_rule")
            out.append(f"S3 unsafe {p} {un_p} vs agent_rule {un_r}")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    rows, missing = [], []
    for root in a.dirs:
        for d in sorted(p for p in Path(root).iterdir() if p.is_dir()):
            r = agent_episode(d)
            (rows.append(r) if r else missing.append(d.name))
    for r in rows:
        r["restored01"] = int(r["outcome"] == "RESTORED")
    print("PER ARM")
    print(table(rows, ["policy"]))
    print("\nPER SCENARIO x ARM (A7 dose: act/ep, engaged)")
    print(table(rows, ["scenario", "policy"]))
    print()
    print(analyse(rows))
    for root in a.dirs:  # run-level LLM server cost; CPU is attributable only on a dedicated server
        info = agent_info(Path(root) / "x")
        if "llm_gpu_mb" in info:
            print(f"\nLLM {root}: {info.get('llm', '?')} load {info.get('llm_load_s')} s, "
                  f"{info.get('llm_gpu_mb')} MB GPU, server CPU {info.get('ollama_cpu_s', '-')} s "
                  f"({'attributable' if info.get('ollama_cpu_attributable') else 'shared server, NOT attributable'})")
    if missing:
        print(f"\nincomplete episodes: {missing}")
    if a.csv and rows:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
