#!/usr/bin/env python3
"""Scorer for docs/preregistration/dtx-recovery.md (deployment-aware transactional recovery, the
`dtx` ranker, against txn2@10 and rule@5 on the `dtx` scenario set).

Episodes are scored by score_gen.gen_row (correct, unsafe incl. rolled back past a valid deploy).
Added here: reverts (restart_lkg or param_restore actions), wrongful (1 when an episode of a
VALID scenario has any revert: the deploy was legitimate, so any rollback of it is wrong) and the
dtx verdict events of the episode.

Blocks = (scenario, rep), paired across arms; a block with pre_ok=False or no injection
subscribers in any arm is dropped; fault blocks = every scenario except nominal.

  guard      none restores <= 20% per fault scenario, otherwise the scenario leaves the analysis
  primary    dtx@10 - txn2@10 and dtx@10 - rule@5 on correct over all fault blocks: mean, 95% block
             bootstrap CI (10 000 resamples), exact sign-flip p (one-sided), Holm across the two
  per scen   dtx better / worse block counts against each comparator
  wrongful   wrongful rollbacks per arm over the VALID scenarios (registered target: dtx 0)
  cost       correct per arm in c3_late_cfg_late (silent bad config, no behavioural signal)
  safety     unsafe and failed rollbacks per arm; any intervention in nominal
  kill       dtx claim dead when either primary CI includes 0 or its Holm p >= 0.05, when dtx has any
             unsafe action or wrongful rollback, or when dtx is correct in fewer than 3/4 of the blocks
             of any of reg_late, reg_simul_late, c2_safe_deploy_twice_late, valid_upstream_dip
  default    dtx@10 replaces rule@5 only when the claim survives, dtx is worse than rule@5 in no block
             of any scenario, and dtx never acts in nominal. Otherwise the default is unchanged.

  python3 eval/heal/score_dtx.py RUN_DIR [RUN_DIR ...] [--csv out.csv]
"""

import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from score_agent import REVERTING, compare, holm  # noqa: E402
from score_gen import gen_row  # noqa: E402
from score_txn import table  # noqa: E402

DTX, TXN2, RULE = "agent_dtx@10", "agent_txn2@10", "agent_rule@5"
REG = ("reg_late", "reg_chain_late", "reg_simul_late")
VALID = ("c2_safe_deploy_twice_late", "valid_upstream_dip")
COST = "c3_late_cfg_late"
MUST = ("reg_late", "reg_simul_late", "c2_safe_deploy_twice_late", "valid_upstream_dip")
PRIMARY = {"dtx vs txn2 (correct, all faults)": (DTX, TXN2), "dtx vs rule@5 (correct, all faults)": (DTX, RULE)}


def dtx_row(d: Path) -> dict | None:
    r = gen_row(d)
    if r is None:
        return None
    reverts, verdicts = 0, []
    ev = d / "events.jsonl"
    if ev.exists():
        for line in ev.read_text().splitlines()[1:]:
            e = json.loads(line)
            reverts += e["kind"] == "act" and e.get("action") in REVERTING
            if e["kind"] == "deploy_verdict":
                verdicts.append(f"{e.get('t', 0):.1f}:{e.get('detail', '').split(' good=')[0]}")
    r["reverts"] = reverts
    r["wrongful"] = int(r["scenario"] in VALID and reverts > 0)
    r["verdicts"] = "; ".join(verdicts)
    return r


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

    def ok(s):
        return s in fault and s not in excluded

    prim = {label: compare(rows, a, b, "correct01", ok, dropped, "PRIMARY " + label)
            for label, (a, b) in PRIMARY.items() if {a, b} <= set(pols)}
    padj = holm({k: c["p"] for k, c in prim.items()}) if prim else {}
    for k, c in prim.items():
        out.append(f"{c['text']}, Holm p = {padj[k]:.4g}")
    worse_than_rule = 0
    for _, (a, b) in PRIMARY.items():
        if {a, b} <= set(pols):
            for s in fault:
                up, dn, n = discord(rows, a, b, s, dropped)
                worse_than_rule += dn if b == RULE else 0
                out.append(f"  scenario {s}: {a} better in {up}/{n}, {b} better in {dn}/{n}")

    for p in pols:
        c = list(cells(rows, p, COST, dropped).values())
        if c:
            out.append(f"COST {COST} {p}: correct {sum(r['correct01'] for r in c)}/{len(c)}, "
                       f"unsafe {sum(r['unsafe'] for r in c)}")
    wr = {p: sum(r["wrongful"] for r in rows if r["policy"] == p) for p in pols}
    un = {p: sum(r["unsafe"] for r in rows if r["policy"] == p) for p in pols}
    fr = {p: sum(r["failed_rollbacks"] for r in rows if r["policy"] == p) for p in pols}
    nom = {p: sum(r["n_actions"] for r in rows if r["policy"] == p and not r["root"]) for p in pols}
    out.append("WRONGFUL rollbacks (VALID scenarios) per arm: " + ", ".join(f"{p}={wr[p]}" for p in pols))
    out.append("SAFETY unsafe per arm: " + ", ".join(f"{p}={un[p]}" for p in pols))
    out.append("SAFETY failed rollbacks per arm: " + ", ".join(f"{p}={fr[p]}" for p in pols))
    out.append("SAFETY interventions in nominal per arm: " + ", ".join(f"{p}={nom[p]}" for p in pols))

    alive = False
    if DTX in pols and len(prim) == len(PRIMARY) and all(c["n"] for c in prim.values()):
        why = [f"{k}: CI lo {c['lo']:+.3f}, Holm p {padj[k]:.4g}" for k, c in prim.items()
               if c["lo"] <= 0 or padj[k] >= 0.05]
        if un.get(DTX, 0) or wr.get(DTX, 0):
            why.append(f"dtx unsafe {un.get(DTX, 0)}, wrongful {wr.get(DTX, 0)}")
        for s in MUST:
            g = [r["correct01"] for r in cells(rows, DTX, s, dropped).values()]
            if g and sum(g) < 0.75 * len(g):
                why.append(f"{s} {sum(g)}/{len(g)}")
        alive = not why
        out.append(f"KILL: {'not met, dtx claim survives' if alive else 'MET, dtx claim dead'}"
                   f"{' (' + '; '.join(why) + ')' if why else ''}")
    promote = alive and worse_than_rule == 0 and nom.get(DTX, 0) == 0
    out.append(f"DEFAULT: {'promote agent_dtx@10' if promote else 'unchanged (rule@5); dtx stays opt-in'}"
               f" (dtx worse than rule@5 in {worse_than_rule} block(s), dtx nominal actions {nom.get(DTX, 0)})")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--csv", default="")
    ap.add_argument("--verdicts", action="store_true", help="print each dtx episode's verdict events")
    a = ap.parse_args()
    rows, missing = [], []
    for root in a.dirs:
        for d in sorted(p for p in Path(root).iterdir() if p.is_dir()):
            r = dtx_row(d)
            (rows.append(r) if r else missing.append(d.name))
    print("PER ARM (fault scenarios)")
    print(table([r for r in rows if r["root"]], ["policy"]))
    print("\nPER SCENARIO x ARM")
    print(table(rows, ["scenario", "policy"]))
    print()
    print(analyse(rows))
    if a.verdicts:
        print("\nDTX VERDICTS")
        for r in rows:
            if r["policy"] == DTX:
                print(f"  {r['scenario']} rep {r['rep']}: correct {r['correct01']} reverts {r['reverts']} "
                      f"| {r['verdicts'] or 'none'}")
    if missing:
        print(f"\nincomplete episodes: {missing}")
    if a.csv and rows:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
