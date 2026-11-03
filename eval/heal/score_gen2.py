#!/usr/bin/env python3
"""Scorer for docs/preregistration/gen-ack-confirm.md (confirmatory gen vs txn2 on the poisoned-LKG
cases, and hash-bound deploy acknowledgement, the `genack` arm, on the `gen2` scenario set).

Episodes are scored by score_gen.gen_row (correct, unsafe incl. rolled back past a valid deploy).
Blocks = (scenario, rep), paired across arms; a block with pre_ok=False or no injection
subscribers in any arm is dropped; fault blocks = every scenario except nominal.

  guard      none restores <= 20% per fault scenario, otherwise the scenario leaves the analysis
  primary A  agent_gen@10 - agent_txn2@10 on correct over the POISON blocks
             (c3_late_cfg_late, c4_chain_late, c5_simul_late)
  primary B  agent_genack@10 - agent_gen@10 on correct over the c2_safe_deploy_twice_late_ack blocks
             Both: mean, 95% block bootstrap CI (10 000 resamples), exact sign-flip p (one-sided),
             Holm across A and B.
  secondary  genack - txn2 and genack - rule@5 on correct over all fault blocks; gen - txn2 on
             correct over all fault blocks (replication of the gen study primary); Holm across three
  cost       correct per arm in c3_late_cfg_late_falseack (registered cost of trusting an ack)
  identity   genack vs gen discordant blocks in the scenarios without an ack part (expected 0:
             with no ack file genack is gen)
  safety     unsafe and failed rollbacks per arm; any intervention in nominal
  kill A     dead when A's CI includes 0, Holm p >= 0.05, or gen is correct in fewer than 3/4 of the
             blocks of any POISON scenario
  kill B     dead when B's CI includes 0, Holm p >= 0.05, or genack has any unsafe action in
             c2_safe_deploy_twice_late_ack
  default    genack@10 replaces rule@5 only when A and B survive, genack - rule@5 has CI lo > 0
             and Holm p < 0.05, genack has no more unsafe than txn2@10 and rule@5, and genack never
             acts in nominal. Otherwise the default is unchanged.

  python3 eval/heal/score_gen2.py RUN_DIR [RUN_DIR ...] [--csv out.csv]
"""

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from score_agent import compare, holm  # noqa: E402
from score_gen import gen_row  # noqa: E402
from score_txn import table  # noqa: E402

GEN, ACK, TXN2, RULE = "agent_gen@10", "agent_genack@10", "agent_txn2@10", "agent_rule@5"
POISON = ("c3_late_cfg_late", "c4_chain_late", "c5_simul_late")
ACK_SCEN, COST_SCEN = "c2_safe_deploy_twice_late_ack", "c3_late_cfg_late_falseack"
NO_ACK = ("c3_late_cfg_late", "c4_chain_late", "c5_simul_late", "c2_safe_deploy_twice_late")
SECONDARY = {"genack vs txn2 (correct, all faults)": (ACK, TXN2),
             "genack vs rule@5 (correct, all faults)": (ACK, RULE),
             "gen vs txn2 (correct, all faults, replication)": (GEN, TXN2)}


def cells(rows, pol, scen, dropped):
    return {r["rep"]: r for r in rows if r["policy"] == pol and r["scenario"] == scen
            and (scen, r["rep"]) not in dropped}


def discord(rows, a, b, scen, dropped):
    ra, rb = cells(rows, a, scen, dropped), cells(rows, b, scen, dropped)
    reps = [k for k in ra if k in rb]
    up = sum(ra[k]["correct01"] > rb[k]["correct01"] for k in reps)
    dn = sum(ra[k]["correct01"] < rb[k]["correct01"] for k in reps)
    return up, dn, len(reps)


def analyse(rows) -> str:
    out = []
    pols = sorted({r["policy"] for r in rows})
    fault = sorted({r["scenario"] for r in rows if r["root"]})
    dropped = {(r["scenario"], r["rep"]) for r in rows if r["root"] and (not r["pre_ok"] or r["inject_subs"] == 0)}
    out.append(f"dropped blocks (pre_ok=False or inject_subs=0): {sorted(dropped) or 'none'}")
    excluded = set()
    for s in fault:
        nr = [r for r in rows if r["policy"] == "none" and r["scenario"] == s]
        if nr:
            k = sum(r["outcome"] == "RESTORED" for r in nr)
            bad = k > 0.2 * len(nr)
            excluded |= {s} if bad else set()
            out.append(f"GUARD none restores {s}: {k}/{len(nr)}{'  EXCLUDED (> 20%)' if bad else ''}")

    def ok(scens):
        return lambda s: s in scens and s in fault and s not in excluded

    prim = {}
    if {GEN, TXN2} <= set(pols):
        prim["A"] = compare(rows, GEN, TXN2, "correct01", ok(POISON), dropped, "PRIMARY A gen vs txn2 (POISON)")
    if {ACK, GEN} <= set(pols):
        prim["B"] = compare(rows, ACK, GEN, "correct01", ok((ACK_SCEN,)), dropped, "PRIMARY B genack vs gen (ack)")
    padj = holm({k: c["p"] for k, c in prim.items()}) if prim else {}
    for k, c in prim.items():
        out.append(f"{c['text']}, Holm p = {padj[k]:.4g}")
    for a, b, scens in ((GEN, TXN2, POISON), (ACK, GEN, (ACK_SCEN,))):
        for s in scens:
            if {a, b} <= set(pols):
                up, dn, n = discord(rows, a, b, s, dropped)
                out.append(f"  scenario {s}: {a} better in {up}/{n}, {b} better in {dn}/{n}")

    sec, ps = {}, {}
    for label, (x, y) in SECONDARY.items():
        if x in pols and y in pols:
            sec[label] = compare(rows, x, y, "correct01", ok(fault), dropped, "SECONDARY " + label)
            ps[label] = sec[label]["p"]
    sadj = holm(ps) if ps else {}
    for label, c in sec.items():
        out.append(f"{c['text']}, Holm p = {sadj[label]:.4g}")

    for p in pols:
        c = list(cells(rows, p, COST_SCEN, dropped).values())
        if c:
            out.append(f"COST {COST_SCEN} {p}: correct {sum(r['correct01'] for r in c)}/{len(c)}, "
                       f"unsafe {sum(r['unsafe'] for r in c)}")
    if {ACK, GEN} <= set(pols):
        ident = [(s, *discord(rows, ACK, GEN, s, dropped)) for s in NO_ACK + ("nominal",)]
        n_dis = sum(u + d for _, u, d, _ in ident)
        out.append(f"IDENTITY genack vs gen without an ack: {n_dis} discordant blocks "
                   + ", ".join(f"{s} +{u}/-{d} of {n}" for s, u, d, n in ident))

    un = {p: sum(r["unsafe"] for r in rows if r["policy"] == p) for p in pols}
    fr = {p: sum(r["failed_rollbacks"] for r in rows if r["policy"] == p) for p in pols}
    nom = {p: sum(r["n_actions"] for r in rows if r["policy"] == p and not r["root"]) for p in pols}
    out.append("SAFETY unsafe per arm: " + ", ".join(f"{p}={un[p]}" for p in pols))
    out.append("SAFETY failed rollbacks per arm: " + ", ".join(f"{p}={fr[p]}" for p in pols))
    out.append("SAFETY interventions in nominal per arm: " + ", ".join(f"{p}={nom[p]}" for p in pols))

    alive = {}
    if "A" in prim and prim["A"]["n"]:
        why = []
        for s in POISON:
            g = [r["correct01"] for r in cells(rows, GEN, s, dropped).values()]
            if g and sum(g) < 0.75 * len(g):
                why.append(f"{s} {sum(g)}/{len(g)}")
        dead = prim["A"]["lo"] <= 0 or padj["A"] >= 0.05 or bool(why)
        alive["A"] = not dead
        out.append(f"KILL A: {'MET, claim A dead' if dead else 'not met, claim A survives'}"
                   f"{' (' + '; '.join(why) + ')' if why else ''}")
    if "B" in prim and prim["B"]["n"]:
        ua = sum(r["unsafe"] for r in cells(rows, ACK, ACK_SCEN, dropped).values())
        dead = prim["B"]["lo"] <= 0 or padj["B"] >= 0.05 or ua > 0
        alive["B"] = not dead
        out.append(f"KILL B: {'MET, claim B dead' if dead else 'not met, claim B survives'} (genack unsafe in "
                   f"{ACK_SCEN}: {ua})")
    g_rule = sec.get("genack vs rule@5 (correct, all faults)")
    promote = (alive.get("A") and alive.get("B") and g_rule is not None and g_rule["n"] and g_rule["lo"] > 0
               and sadj.get("genack vs rule@5 (correct, all faults)", 1.0) < 0.05
               and un.get(ACK, 0) <= min(un.get(TXN2, 0), un.get(RULE, 0)) and nom.get(ACK, 0) == 0)
    out.append(f"DEFAULT: {'promote agent_genack@10' if promote else 'unchanged (rule@5); gen and genack stay opt-in'}")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    rows, missing = [], []
    for root in a.dirs:
        for d in sorted(p for p in Path(root).iterdir() if p.is_dir()):
            r = gen_row(d)
            (rows.append(r) if r else missing.append(d.name))
    print("PER ARM (fault scenarios)")
    print(table([r for r in rows if r["root"]], ["policy"]))
    print("\nPER SCENARIO x ARM")
    print(table(rows, ["scenario", "policy"]))
    print()
    print(analyse(rows))
    if missing:
        print(f"\nincomplete episodes: {missing}")
    if a.csv and rows:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
