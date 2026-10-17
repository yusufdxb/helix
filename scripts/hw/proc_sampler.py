#!/usr/bin/env python3
"""Per-process CPU and RSS sampler for HELIX runtime benchmarks.

Reads /proc directly (no psutil) so it runs on a bare Jetson. Every
``--period`` seconds it finds processes whose command line matches
``--match`` and writes one JSON line per process:

    {"t": <unix s>, "pid": .., "name": .., "cpu_pct": .., "rss_kb": ..,
     "threads": .., "vol_ctx": .., "nonvol_ctx": ..}

cpu_pct is the utime+stime delta over the wall-clock delta for that pid,
in percent of ONE core (so a fully busy single thread reads 100). The first
sample of a pid has no delta and is written with cpu_pct null. A final
summary line {"summary": {...}} gives per-name mean/max CPU and RSS over the
samples after ``--warmup`` seconds.

Usage:
    proc_sampler.py --out samples.jsonl --duration 1800 --match helix_
"""
from __future__ import annotations

import argparse
import json
import os
import re
import statistics
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")


def _cmdline(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def _stat(pid: int):
    try:
        with open(f"/proc/{pid}/stat") as f:
            raw = f.read()
    except OSError:
        return None
    # comm may contain spaces; fields after the closing paren are fixed.
    rest = raw[raw.rindex(")") + 2:].split()
    utime, stime = int(rest[11]), int(rest[12])
    threads = int(rest[17])
    return utime + stime, threads


def _status(pid: int):
    rss = vol = nonvol = None
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1])
                elif line.startswith("voluntary_ctxt_switches:"):
                    vol = int(line.split()[1])
                elif line.startswith("nonvoluntary_ctxt_switches:"):
                    nonvol = int(line.split()[1])
    except OSError:
        return None
    return rss, vol, nonvol


def _name(cmd: str, pattern: re.Pattern) -> str:
    """The first matching token, e.g. helix_anomaly_detector."""
    for tok in cmd.split():
        base = os.path.basename(tok)
        if pattern.search(base):
            return base
    return cmd.split()[0] if cmd else "?"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--period", type=float, default=5.0)
    ap.add_argument("--warmup", type=float, default=30.0)
    ap.add_argument("--match", default="helix_")
    ap.add_argument("--exclude", default="proc_sampler|ros2 launch|bag record|/bin/sh|bash ")
    args = ap.parse_args()
    pat = re.compile(args.match)
    excl = re.compile(args.exclude)
    prev = {}
    per_name = {}
    t0 = time.time()
    with open(args.out, "w") as out:
        while time.time() - t0 < args.duration:
            now = time.time()
            for entry in os.listdir("/proc"):
                if not entry.isdigit():
                    continue
                pid = int(entry)
                cmd = _cmdline(pid)
                if not cmd or not pat.search(cmd) or excl.search(cmd):
                    continue
                st = _stat(pid)
                ss = _status(pid)
                if st is None or ss is None:
                    continue
                ticks, threads = st
                rss, vol, nonvol = ss
                cpu = None
                if pid in prev:
                    pticks, pt = prev[pid]
                    dt = now - pt
                    if dt > 0:
                        cpu = 100.0 * (ticks - pticks) / CLK_TCK / dt
                prev[pid] = (ticks, now)
                name = _name(cmd, pat)
                rec = {"t": round(now, 3), "pid": pid, "name": name, "cpu_pct": cpu,
                       "rss_kb": rss, "threads": threads, "vol_ctx": vol,
                       "nonvol_ctx": nonvol}
                out.write(json.dumps(rec) + "\n")
                if cpu is not None and now - t0 >= args.warmup:
                    agg = per_name.setdefault(name, {"cpu": [], "rss": []})
                    agg["cpu"].append(cpu)
                    agg["rss"].append(rss)
            out.flush()
            time.sleep(max(0.0, args.period - (time.time() - now)))
        summary = {}
        for name, agg in sorted(per_name.items()):
            summary[name] = {
                "n": len(agg["cpu"]),
                "cpu_mean": round(statistics.fmean(agg["cpu"]), 3),
                "cpu_max": round(max(agg["cpu"]), 3),
                "rss_mean_kb": round(statistics.fmean(agg["rss"]), 1),
                "rss_max_kb": max(agg["rss"]),
            }
        out.write(json.dumps({"summary": summary}) + "\n")


if __name__ == "__main__":
    main()
