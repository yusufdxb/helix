#!/usr/bin/env python3
"""Scorer for docs/preregistration/gen-snapshots.md (versioned config snapshots, the `gen`
ranker, against txn2@10 and rule@5 on the `gen` scenario set).

Episodes are scored by score_agent.agent_episode. Added here:
  correct    strict_restored (RESTORED and the poisoned-config marker is not live at the end)
             and unsafe == 0 and, where the scenario has a markerwatch (c4_chain_late), the
             wanted config is live at the end (want_live_end)
  unsafe     score_agent's count, plus one when the arm reverted (restart_lkg or param_restore)
             and the wanted config is not live at the end (it rolled back past the valid deploy)

Analysis (blocks = (scenario, rep), paired across arms; a block with pre_ok=False or no
injection subscribers in any arm is dropped; fault blocks = every scenario except nominal):
  guard      none restores <= 20% per fault scenario, otherwise the scenario leaves the paired
             analysis
  primary    agent_gen@10 - agent_txn2@10 on correct over the fault blocks: mean, 95% block
             bootstrap CI (10 000 resamples), exact sign-flip p
  secondary  gen@10 - rule@5 on correct; txn2@10 - rule@5 on correct (replication); gen@10 -
             txn2@10 on downtime_s (lower better); Holm across the three
  safety     unsafe and failed rollbacks per arm; any intervention in nominal
  kill       the gen claim is dead when the primary CI includes 0 or gen is correct in fewer
             than 3/4 of the c3_late_cfg_late or the c4_chain_late blocks
  default    gen replaces rule@5 only when the primary CI lo > 0, gen - rule@5 has CI lo > 0
             and Holm p < 0.05, gen has no more unsafe than either comparator, and gen never
             acts in nominal. Otherwise the default is unchanged.

  python3 eval/heal/score_gen.py RUN_DIR [RUN_DIR ...] [--csv out.csv]
"""

import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from score_agent import REVERTING, agent_episode, compare, holm  # noqa: E402
from score_txn import table  # noqa: E402

GEN, TXN2, RULE = "agent_gen@10", "agent_txn2@10", "agent_rule@5"
PRIMARY = (GEN, TXN2)
SECONDARY = {"gen vs rule@5 (correct)": (GEN, RULE, "correct01", False),
             "txn2 vs rule@5 (correct, replication)": (TXN2, RULE, "correct01", False),
             "gen vs txn2 (downtime_s)": (GEN, TXN2, "downtime_s", True)}
KILL_SCEN = ("c3_late_cfg_late", "c4_chain_late")


def gen_row(d: Path) -> dict | None:
    r = agent_episode(d)
    if r is None:
        return None
    inj = (json.loads((d / "probe.json").read_text()).get("injector") or {})
    want = inj.get("want_live_end")
    reverted = 0
    ev = d / "events.jsonl"
    if ev.exists():
        for line in ev.read_text().splitlines()[1:]:
            e = json.loads(line)
            reverted += e["kind"] == "act" and e.get("action") in REVERTING
    r["want_live_end"] = want
    r["past_valid"] = int(want is False and reverted > 0)
    r["unsafe"] += r["past_valid"]
    r["correct01"] = int(bool(r["strict_restored"]) and r["unsafe"] == 0 and want is not False)
    r["restored01"] = int(r["outcome"] == "RESTORED")
    return r


def analyse(rows) -> str:
    out = []
    pols = sorted({r["policy"] for r in rows})
    fault = sorted({r["scenario"] for r in rows if r["root"]})
    dropped = {(r["scenario"], r["rep"]) for r in rows if r["root"] and (not r["pre_ok"] or r["inject_subs"] == 0)}
    if dropped:
        out.append(f"dropped blocks (pre_ok=False or inject_subs=0): {sorted(dropped)}")
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

    a, b = PRIMARY
    prim = None
    if a in pols and b in pols:
        prim = compare(rows, a, b, "correct01", scen_ok, dropped, "PRIMARY correct")
        out.append(prim["text"])
        for s in fault:
            ra = {r["rep"]: r["correct01"] for r in rows if r["policy"] == a and r["scenario"] == s}
            rb = {r["rep"]: r["correct01"] for r in rows if r["policy"] == b and r["scenario"] == s}
            reps = [k for k in ra if k in rb and (s, k) not in dropped]
            up = sum(ra[k] > rb[k] for k in reps)
            dn = sum(ra[k] < rb[k] for k in reps)
            out.append(f"  scenario {s}{'' if scen_ok(s) else ' (excluded)'}: {a} better in {up}/{len(reps)}, "
                       f"{b} better in {dn}/{len(reps)}")
    sec, ps = {}, {}
    for label, (x, y, key, low) in SECONDARY.items():
        if x in pols and y in pols:
            sec[label] = compare(rows, x, y, key, scen_ok, dropped, "SECONDARY " + label, lower_better=low)
            ps[label] = sec[label]["p"]
    adj = holm(ps) if ps else {}
    for label, c in sec.items():
        out.append(f"{c['text']}, Holm p = {adj[label]:.4g}")

    un = {p: sum(r["unsafe"] for r in rows if r["policy"] == p) for p in pols}
    fr = {p: sum(r["failed_rollbacks"] for r in rows if r["policy"] == p) for p in pols}
    nom = {p: sum(r["n_actions"] for r in rows if r["policy"] == p and not r["root"]) for p in pols}
    out.append("SAFETY unsafe per arm: " + ", ".join(f"{p}={un[p]}" for p in pols))
    out.append("SAFETY failed rollbacks per arm: " + ", ".join(f"{p}={fr[p]}" for p in pols))
    out.append("SAFETY interventions in nominal per arm: " + ", ".join(f"{p}={nom[p]}" for p in pols))

    if prim and prim["n"]:
        kill_s = []
        for s in KILL_SCEN:
            g = [r["correct01"] for r in rows if r["policy"] == GEN and r["scenario"] == s
                 and (s, r["rep"]) not in dropped]
            if g and sum(g) < 0.75 * len(g):
                kill_s.append(f"{s} {sum(g)}/{len(g)}")
        dead = prim["lo"] <= 0 or bool(kill_s)
        out.append(f"KILL (primary CI includes 0, or gen < 3/4 on {'/'.join(KILL_SCEN)}): "
                   f"{'MET, gen claim dead' if dead else 'not met, gen claim survives'}"
                   f"{' (' + '; '.join(kill_s) + ')' if kill_s else ''}")
        g_rule = sec.get("gen vs rule@5 (correct)")
        promote = (not dead and g_rule is not None and g_rule["n"] and g_rule["lo"] > 0
                   and adj.get("gen vs rule@5 (correct)", 1.0) < 0.05
                   and un.get(GEN, 0) <= min(un.get(TXN2, 0), un.get(RULE, 0)) and nom.get(GEN, 0) == 0)
        out.append(f"DEFAULT: {'promote agent_gen@10' if promote else 'unchanged (rule@5); gen stays opt-in'}")
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
