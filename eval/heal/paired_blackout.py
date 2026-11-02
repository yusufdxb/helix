#!/usr/bin/env python3
"""Co-primary analysis of docs/preregistration/2026-11-02-humanoid-real.md.

Pairs arm A and arm B on every restorable fault block (scenario, rep) of a score.py CSV and
tests the per-block difference in blackout_s (longest gap on any judged topic after the
injection). Pairs with pre_ok=False or inject_subs=0 in either arm are dropped.
Reports the median difference, a seeded percentile bootstrap 95% CI of the mean difference,
and an exact two-sided sign-flip p-value.

Usage: paired_blackout.py SCORE.csv [--a helix] [--b watchdog]
"""
import argparse
import csv
import random
import statistics


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--a", default="helix")
    ap.add_argument("--b", default="watchdog")
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))
    bad = {(r["scenario"], r["rep"]) for r in rows if r["root"] and (r["pre_ok"] != "True" or r["inject_subs"] == "0")}
    cell = {}
    for r in rows:
        key = (r["scenario"], r["rep"])
        if r["root"] and r["restorable"] == "True" and key not in bad and r["policy"] in (a.a, a.b):
            cell.setdefault(key, {})[r["policy"]] = float(r["blackout_s"])
    d = [v[a.a] - v[a.b] for v in cell.values() if len(v) == 2]
    if not d:
        print("no paired blocks")
        return
    n = len(d)
    obs = abs(sum(d))
    if n <= 20:
        ge = sum(1 for m in range(2 ** n) if abs(sum(x if (m >> i) & 1 else -x for i, x in enumerate(d))) >= obs - 1e-12)
        p, how = ge / 2 ** n, "exact"
    else:
        rng = random.Random(0)
        ge = sum(1 for _ in range(100000) if abs(sum(x if rng.random() < 0.5 else -x for x in d)) >= obs - 1e-12)
        p, how = (ge + 1) / 100001, "Monte Carlo 1e5"
    rng = random.Random(1)
    boot = sorted(statistics.fmean(rng.choices(d, k=n)) for _ in range(10000))
    lo, hi = boot[249], boot[9749]
    print(f"BLACKOUT {a.a} - {a.b}: {n} paired restorable fault blocks, dropped {len(bad)}; "
          f"median {statistics.median(d):+.3f} s, mean {statistics.fmean(d):+.3f} s, "
          f"95% bootstrap CI [{lo:+.3f}, {hi:+.3f}]; sign-flip p ({how}, two-sided) = {p:.3g}; "
          f"{a.a} shorter in {sum(1 for x in d if x < 0)}/{n}")
    kill = not (hi < 0)
    print(f"KILL CRITERION (CI upper bound >= 0): {'MET, latency claim dead' if kill else 'not met, latency claim survives'}")


if __name__ == "__main__":
    main()
