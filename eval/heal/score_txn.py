#!/usr/bin/env python3
"""Scorer for docs/preregistration/txn-baseline.md (HELIX rule ranker vs a transactional
restart-and-rollback baseline on the `txn` scenario set).

Episodes are scored by score_agent.agent_episode. Added here:
  correct    strict_restored (RESTORED and the poisoned-config marker is not live at the end)
             and unsafe == 0
  family     C1 multi-cause, C2 legitimate deploy then unrelated crash, C3 delayed config
             failure that can poison the LKG snapshot

Analysis (blocks = (scenario, rep), paired across arms; a block with pre_ok=False or no
injection subscribers in any arm is dropped; fault blocks = every scenario except nominal):
  guards     agent_oracle@10 correct >= 90% on restorable fault episodes; none restores
             <= 20% per fault scenario, otherwise the scenario leaves the paired analysis
  primary    agent_rule@5 - agent_txn@10 on correct over the fault blocks: mean, 95% block
             bootstrap CI (10 000 resamples), exact sign-flip p
  verdict    beats (CI lo > 0) / loses (CI hi < 0) / trade-off (at least one scenario favours
             each arm: >= 80% of its blocks discordant that way) / equivalent (CI inside +/-0.10
             and no scenario favours either arm) / inconclusive
  secondary  selection at matched window (rule@10 - txn@10, rule@5 - txn@5), window (rule@10 -
             rule@5, txn@5 - txn@10), two-strike (txn2@10 - txn@10) on correct, Holm across the
             five; per-family and per-scenario primary (descriptive)
  safety     unsafe and failed rollbacks per arm and family; any intervention in nominal

Attribution run (--attrib, run B on --set agent_withheld): where did the withheld-study gain of
the rule ranker over v3 helix come from? rollback = agent_txn@5 - helix, selection =
agent_rule@5 - agent_txn@5, on correct and on restored, Holm across the four; share of the
rule - helix gain that plain rollback reproduces = (txn - helix) / (rule - helix).

  python3 eval/heal/score_txn.py RUN_DIR [RUN_DIR ...] [--attrib] [--csv out.csv]
"""

import argparse
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from score_agent import agent_episode, compare, frac, holm, mean  # noqa: E402

PRIMARY = ("agent_rule@5", "agent_txn@10")
SECONDARY = {"selection at H=10 (rule vs txn)": ("agent_rule@10", "agent_txn@10"),
             "selection at H=5 (rule vs txn)": ("agent_rule@5", "agent_txn@5"),
             "window, rule (H=10 vs 5)": ("agent_rule@10", "agent_rule@5"),
             "window, txn (H=5 vs 10)": ("agent_txn@5", "agent_txn@10"),
             "two-strike (txn2 vs txn)": ("agent_txn2@10", "agent_txn@10")}
FAVOUR = 0.8  # a scenario favours an arm when >= 80% of its paired blocks are discordant that way
ORACLE = "agent_oracle@10"
EQUIV = 0.10  # kill margin, see the preregistration
ATTRIB = {"rollback (txn@5 - helix)": ("agent_txn@5", "helix"),
          "selection (rule@5 - txn@5)": ("agent_rule@5", "agent_txn@5")}


def family(scen: str) -> str:
    return scen.split("_", 1)[0].upper() if scen[:2] in ("c1", "c2", "c3") else "nominal"


def table(rows, by):
    g: dict[tuple, list] = defaultdict(list)
    for r in rows:
        g[tuple(r[k] for k in by)].append(r)
    hdr = by + ["n", "correct", "strict", "restored", "repair_ok", "unsafe", "rollbacks", "failed_rb",
                "lkg_commits", "act/ep", "engaged", "downtime_s", "cens", "ranker_ms", "cpu", "rss_mb"]
    out = ["\t".join(hdr)]
    for k, rs in sorted(g.items()):
        out.append("\t".join(list(map(str, k)) + [
            str(len(rs)), frac(r["correct01"] for r in rs), frac(r["strict_restored"] for r in rs),
            frac(r["outcome"] == "RESTORED" for r in rs), frac(r["repair_ok"] for r in rs),
            str(sum(r["unsafe"] for r in rs)), str(sum(r["rollbacks"] for r in rs)),
            str(sum(r["failed_rollbacks"] for r in rs)), mean([r["lkg_commits"] for r in rs]),
            mean([r["n_actions"] for r in rs]), frac(r["engaged"] for r in rs),
            mean([r["downtime_s"] for r in rs]), str(sum(bool(r["downtime_censored"]) for r in rs)),
            mean([r["ranker_ms_mean"] for r in rs], 4), mean([r["healer_cpu"] for r in rs], 3),
            mean([r["healer_rss_mb"] for r in rs], 1)]))
    return "\n".join(out)


def analyse(rows) -> str:
    out = []
    pols = sorted({r["policy"] for r in rows})
    fault = sorted({r["scenario"] for r in rows if r["root"]})
    dropped = {(r["scenario"], r["rep"]) for r in rows if r["root"] and (not r["pre_ok"] or r["inject_subs"] == 0)}
    if dropped:
        out.append(f"dropped blocks (pre_ok=False or inject_subs=0): {sorted(dropped)}")

    # Guards.
    orc = [r for r in rows if r["policy"] == ORACLE and r["root"] and r["restorable"]]
    if orc:
        k = sum(r["correct01"] for r in orc)
        out.append(f"GUARD oracle correct on restorable fault episodes {k}/{len(orc)} = {k / len(orc):.3f} "
                   f"({'PASS' if k >= 0.9 * len(orc) else 'FAIL'}, need >= 0.90)")
    excluded = set()
    for s in fault:
        nr = [r for r in rows if r["policy"] == "none" and r["scenario"] == s]
        if nr:
            k = sum(r["outcome"] == "RESTORED" for r in nr)
            bad = k > 0.2 * len(nr)
            excluded |= {s} if bad else set()
            out.append(f"GUARD none restores {s}: {k}/{len(nr)}{'  EXCLUDED (> 20%)' if bad else ''}")
    def scen_ok(s):
        return s in fault and s not in excluded

    # Primary.
    a, b = PRIMARY
    if a in pols and b in pols:
        c = compare(rows, a, b, "correct01", scen_ok, dropped, "PRIMARY correct")
        out.append(c["text"])
        fav = {a: [], b: []}
        for s in fault:
            if not scen_ok(s):
                continue
            ra = {r["rep"]: r["correct01"] for r in rows if r["policy"] == a and r["scenario"] == s}
            rb = {r["rep"]: r["correct01"] for r in rows if r["policy"] == b and r["scenario"] == s}
            reps = [k for k in ra if k in rb and (s, k) not in dropped]
            if not reps:
                continue
            up = sum(ra[k] > rb[k] for k in reps)
            dn = sum(ra[k] < rb[k] for k in reps)
            fav[a] += [s] if up >= FAVOUR * len(reps) else []
            fav[b] += [s] if dn >= FAVOUR * len(reps) else []
            out.append(f"  scenario {s}: {a} better in {up}/{len(reps)}, {b} better in {dn}/{len(reps)}")
        out.append(f"  scenarios favouring {a}: {fav[a] or '-'}; favouring {b}: {fav[b] or '-'}")
        if c["n"]:
            if fav[a] and fav[b]:
                verdict = ("trade-off: each arm wins scenarios the other loses, so neither selection "
                           "nor plain rollback dominates (see per-scenario lines)")
            elif c["lo"] > 0:
                verdict = "HELIX rule ranker beats the transactional baseline"
            elif c["hi"] < 0:
                verdict = "transactional baseline beats the HELIX rule ranker"
            elif -EQUIV <= c["lo"] and c["hi"] <= EQUIV and not fav[a] and not fav[b]:
                verdict = f"equivalent within +/-{EQUIV:.2f}: selection adds nothing measurable over rollback"
            else:
                verdict = "inconclusive (CI includes 0 and is wider than the equivalence margin)"
            out.append(f"PRIMARY verdict: {verdict}")
        for fam in ("C1", "C2", "C3"):
            f = compare(rows, a, b, "correct01", lambda s, fam=fam: scen_ok(s) and family(s) == fam, dropped,
                        f"  family {fam} correct")
            out.append(f["text"] + " (descriptive)")

    # Secondaries with Holm.
    res = {lab: compare(rows, x, y, "correct01", scen_ok, dropped, f"SECONDARY {lab} correct")
           for lab, (x, y) in SECONDARY.items() if x in pols and y in pols}
    if res:
        adj = holm({k: v["p"] for k, v in res.items()})
        for k, v in res.items():
            out.append(f"{v['text']}; Holm p = {adj[k]:.4g}")
    for lab, key in (("downtime_s", "downtime_s"), ("failed_rollbacks", "failed_rollbacks")):
        if a in pols and b in pols:
            out.append(compare(rows, a, b, key, scen_ok, dropped, f"EXPLORATORY {lab}", lower_better=True)["text"])

    # Safety and dose, per arm x family.
    out.append("\nSAFETY / DOSE per arm x family (unsafe, failed rollbacks, act/ep)")
    for p in pols:
        cells = []
        for fam in ("nominal", "C1", "C2", "C3"):
            rs = [r for r in rows if r["policy"] == p and family(r["scenario"]) == fam]
            if rs:
                cells.append(f"{fam}: unsafe {sum(r['unsafe'] for r in rs)} failed_rb "
                             f"{sum(r['failed_rollbacks'] for r in rs)} act/ep "
                             f"{statistics.fmean(r['n_actions'] for r in rs):.2f}")
        out.append(f"{p}: " + "; ".join(cells))

    # Model-reconsideration check: where does the best deterministic arm trail the oracle?
    det = [p for p in pols if p.startswith("agent_") and p != ORACLE]
    out.append("\nORACLE GAP per scenario (oracle correct - best deterministic correct; 'fixed' = the one "
               "deterministic arm with the highest correct over all fault scenarios)")
    agg = {p: [r["correct01"] for r in rows if r["policy"] == p and r["root"]
               and (r["scenario"], r["rep"]) not in dropped and scen_ok(r["scenario"])] for p in det}
    fixed = max((p for p in det if agg[p]), key=lambda p: statistics.fmean(agg[p]), default="")
    for s in fault:
        def rate(p):
            rs = [r for r in rows if r["policy"] == p and r["scenario"] == s and (s, r["rep"]) not in dropped]
            return sum(r["correct01"] for r in rs) / len(rs) if rs else None
        best = max(((rate(p), p) for p in det if rate(p) is not None), default=(None, ""))
        o = rate(ORACLE)
        if o is not None and best[0] is not None:
            fx = rate(fixed) if fixed else None
            out.append(f"{s}: oracle {o:.2f}, best deterministic {best[1]} {best[0]:.2f}, gap {o - best[0]:+.2f}"
                       + (f"; fixed {fixed} {fx:.2f}, gap {o - fx:+.2f}" if fx is not None else ""))
    return "\n".join(out)


def attrib(rows) -> str:
    out = []
    fault = {r["scenario"] for r in rows if r["root"]}
    dropped = {(r["scenario"], r["rep"]) for r in rows if r["root"] and (not r["pre_ok"] or r["inject_subs"] == 0)}
    if dropped:
        out.append(f"dropped blocks (pre_ok=False or inject_subs=0): {sorted(dropped)}")
    def scen_ok(s):
        return s in fault
    res = {}
    for lab, (x, y) in ATTRIB.items():
        for key in ("correct01", "restored01"):
            res[f"{lab} {key}"] = compare(rows, x, y, key, scen_ok, dropped, f"ATTRIB {lab} {key}")
    adj = holm({k: v["p"] for k, v in res.items()})
    for k, v in res.items():
        out.append(f"{v['text']}; Holm p = {adj[k]:.4g}")
    for key in ("correct01", "restored01"):
        tot = compare(rows, "agent_rule@5", "helix", key, scen_ok, dropped, f"TOTAL rule@5 - helix {key}")
        rb = res[f"rollback (txn@5 - helix) {key}"]
        out.append(tot["text"])
        if tot["n"] and rb["n"] and tot["mean"]:
            out.append(f"  share of the rule - helix gain on {key} reproduced by plain rollback: "
                       f"{rb['mean'] / tot['mean']:.2f}")
    out.append("\nSAFETY / DOSE per arm (unsafe on all episodes, failed rollbacks, act/ep)")
    for p in sorted({r["policy"] for r in rows}):
        rs = [r for r in rows if r["policy"] == p]
        out.append(f"{p}: unsafe {sum(r['unsafe'] for r in rs)} failed_rb {sum(r['failed_rollbacks'] for r in rs)} "
                   f"act/ep {statistics.fmean(r['n_actions'] for r in rs):.2f}")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--csv", default="")
    ap.add_argument("--attrib", action="store_true", help="run B attribution scoring")
    a = ap.parse_args()
    rows, missing = [], []
    for root in a.dirs:
        for d in sorted(p for p in Path(root).iterdir() if p.is_dir()):
            r = agent_episode(d)
            (rows.append(r) if r else missing.append(d.name))
    for r in rows:
        r["correct01"] = int(bool(r["strict_restored"]) and r["unsafe"] == 0)
        r["restored01"] = int(r["outcome"] == "RESTORED")
        r["family"] = family(r["scenario"])
    print("PER ARM (fault scenarios)")
    print(table([r for r in rows if r["root"]], ["policy"]))
    print("\nPER SCENARIO x ARM")
    print(table(rows, ["scenario", "policy"]))
    print()
    print(attrib(rows) if a.attrib else analyse(rows))
    if missing:
        print(f"\nincomplete episodes: {missing}")
    if a.csv and rows:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
