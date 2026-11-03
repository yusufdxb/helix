#!/usr/bin/env python3
"""Stage 0 check for an attach= spec: which live process would HELIX adopt for each node?

Mirrors helix_healer::find_process (process.cpp): a match is a process whose NUL-joined-by-space
cmdline contains the pattern and does not contain "helix_healer"; zombies are skipped; among the
matches HELIX adopts the newest one that has no matching child. Read-only, stdlib only, so it runs
on the Orin as is:

  python3 attach_check.py go2_orin.spec            # exit 0 when every adopted node matches exactly once

Per node: OK (1 match), MULTI (n > 1 matches: adoption is ambiguous, tighten the pattern),
ABSENT (0 matches: not running, never judged).
"""

import argparse
import os
import sys


def attach_patterns(spec_text: str) -> list[tuple[str, str, bool]]:
    """(node, pattern, act) for every node line with attach=."""
    out = []
    for raw in spec_text.splitlines():
        tok = raw.split("#", 1)[0].split()
        if len(tok) < 2 or tok[0] != "node":
            continue
        kv = dict(t.split("=", 1) for t in tok[2:] if "=" in t)
        if "attach" in kv:
            out.append((tok[1], kv["attach"], kv.get("act", "1") not in ("0", "false")))
    return out


def processes(proc: str = "/proc") -> list[dict]:
    self_pid = os.getpid()
    out = []
    for name in os.listdir(proc):
        if not name.isdigit() or int(name) == self_pid:
            continue
        try:
            with open(f"{proc}/{name}/cmdline", "rb") as f:
                argv = [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
            with open(f"{proc}/{name}/stat") as f:
                stat = f.read()
        except OSError:
            continue
        rest = stat[stat.rfind(")") + 2:].split()
        if not argv or len(rest) < 20 or rest[0] in ("Z", "X"):
            continue
        out.append({"pid": int(name), "ppid": int(rest[1]), "start": float(rest[19]), "cmd": " ".join(argv)})
    return out


def adopt(pattern: str, procs: list[dict]) -> tuple[int, dict | None]:
    hits = [p for p in procs if pattern in p["cmd"] and "helix_healer" not in p["cmd"]]
    parents = {h["ppid"] for h in hits}
    leaves = [h for h in hits if h["pid"] not in parents]
    return len(hits), max(leaves, key=lambda h: h["start"], default=None)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("spec")
    ap.add_argument("--proc", default="/proc")
    a = ap.parse_args()
    procs = processes(a.proc)
    bad = 0
    for node, pat, act in attach_patterns(open(a.spec).read()):
        n, best = adopt(pat, procs)
        verdict = "OK" if n == 1 else ("ABSENT" if n == 0 else "MULTI")
        bad += verdict == "MULTI"
        cmd = best["cmd"][:100] if best else ""
        print(f"{verdict:6} {node:20} act={int(act)} matches={n} pid={best['pid'] if best else '-'} {cmd}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
