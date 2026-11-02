#!/usr/bin/env python3
"""Turn developmental agent episodes into ranker training data, then fit the MLP ranker.

Every `act` event carries the evidence features (`x=`) and signature (`sig=`) the ranker saw;
the next verdict for the same node (verified, verify_failed, action_failed) labels it. From
those rows this script writes

  --history OUT   the HistoryRanker warm-start log ("sig fault action ok" per line)
  --mlp OUT       MlpRanker weights ("in hidden", W1, b1, W2, b2, row major)

The MLP scores (features, action one-hot) -> P(repair verified). Before the final fit it runs
leave-one-scenario-out validation and prints, per held-out scenario, whether the top-ranked
feasible action matches one that verified, so the gain over rule order is visible before the
weights are used. Plumbing episodes (set agent_plumbing) are refused: they are never training
or history data.

Usage: train_ranker.py DIR [DIR ...] --history H.log --mlp W.txt [--hidden 16] [--epochs 400]
"""
import argparse
import json
import re
from collections import defaultdict
from pathlib import Path

ACTIONS = ["restart", "lifecycle_reset", "param_restore", "restart_lkg"]  # repair_actions() order
VERDICTS = {"verified", "verify_failed", "action_failed"}
SIG = re.compile(r"\bsig=(\S+)")
X = re.compile(r"\bx=(\S+)")
CANDS = re.compile(r"\bcands=(\S+)")


def rows_from(ep: Path) -> list[dict]:
    meta = json.loads((ep / "meta.json").read_text())
    if meta.get("set") == "agent_plumbing":
        raise SystemExit(f"{ep}: plumbing episodes are never training data")
    ev = [json.loads(line) for line in (ep / "events.jsonl").read_text().splitlines()[1:]]
    out = []
    for i, e in enumerate(ev):
        if e["kind"] != "act" or e.get("action") not in ACTIONS:
            continue
        sig, x = SIG.search(e.get("detail", "")), X.search(e.get("detail", ""))
        if not sig or not x:
            continue
        v = next((w for w in ev[i + 1:] if w["kind"] in VERDICTS and w.get("node") == e["node"]), None)
        if v is None:
            continue  # episode ended before the verdict
        c = CANDS.search(e["detail"])
        out.append(dict(scenario=meta["scenario"], episode=ep.name, node=e["node"], fault=e["fault"],
                        sig=sig.group(1), x=[float(t) for t in x.group(1).split(",")], action=e["action"],
                        cands=c.group(1).split(",") if c else [e["action"]], ok=int(v["kind"] == "verified")))
    return out


def load(dirs: list[str]) -> list[dict]:
    rows = []
    for d in dirs:
        for ep in sorted(Path(d).iterdir()):
            if (ep / "events.jsonl").exists() and (ep / "meta.json").exists():
                rows += rows_from(ep)
    return rows


def fit(rows, hidden, epochs, seed, dev):
    import torch  # lazy: keeps this module importable without torch

    torch.manual_seed(seed)
    xs = torch.tensor([r["x"] + [float(a == r["action"]) for a in ACTIONS] for r in rows], device=dev)
    ys = torch.tensor([float(r["ok"]) for r in rows], device=dev)
    net = torch.nn.Sequential(torch.nn.Linear(xs.shape[1], hidden), torch.nn.ReLU(), torch.nn.Linear(hidden, 1)).to(dev)
    opt = torch.optim.Adam(net.parameters(), lr=1e-2, weight_decay=1e-3)
    for ep in range(epochs):
        opt.zero_grad()
        loss = torch.nn.functional.binary_cross_entropy_with_logits(net(xs).squeeze(1), ys)
        loss.backward()
        opt.step()
    return net, float(loss)


def top_action(net, r, dev):
    import torch

    with torch.no_grad():
        xs = torch.tensor([r["x"] + [float(a == c) for a in ACTIONS] for c in r["cands"]], device=dev)
        s = net(xs).squeeze(1).tolist()
    return r["cands"][max(range(len(s)), key=lambda i: (s[i], -i))]  # ties keep rule order


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--history", required=True)
    ap.add_argument("--mlp", required=True)
    ap.add_argument("--hidden", type=int, default=16)
    ap.add_argument("--epochs", type=int, default=400)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    import torch

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    rows = load(a.dirs)
    if not rows:
        raise SystemExit("no labelled act events found")
    n_ok = sum(r["ok"] for r in rows)
    print(f"rows {len(rows)} ({n_ok} verified) from {len({r['episode'] for r in rows})} episodes, device {dev}"
          f"{' ' + torch.cuda.get_device_name(0) if dev == 'cuda' else ''}")
    by_act = defaultdict(lambda: [0, 0])
    for r in rows:
        by_act[r["action"]][r["ok"]] += 1
    print("per action (fail, ok):", dict(by_act))

    Path(a.history).write_text("".join(f"{r['sig']} {r['fault']} {r['action']} {r['ok']}\n" for r in rows))
    print(f"history: {len(rows)} lines -> {a.history}")

    # Leave-one-scenario-out: does the MLP's first choice verify more often than rule order's?
    # Only first decisions of an incident with an observed outcome for the chosen action count.
    good = defaultdict(set)  # (scenario, node, sig) -> actions observed to verify in any episode
    for r in rows:
        if r["ok"]:
            good[(r["scenario"], r["node"], r["sig"])].add(r["action"])
    hits = defaultdict(lambda: [0, 0, 0])  # scenario -> [n, mlp_hit, rule_hit]
    for s in sorted({r["scenario"] for r in rows}):
        tr = [r for r in rows if r["scenario"] != s]
        te = [r for r in rows if r["scenario"] == s and len(r["cands"]) > 1]
        if not tr or not te:
            continue
        net, _ = fit(tr, a.hidden, a.epochs, a.seed, dev)
        for r in te:
            g = good[(r["scenario"], r["node"], r["sig"])]
            if not g:
                continue
            h = hits[s]
            h[0] += 1
            h[1] += top_action(net, r, dev) in g
            h[2] += r["cands"][0] in g
    tot = [sum(h[i] for h in hits.values()) for i in range(3)]
    for s, h in sorted(hits.items()):
        print(f"  LOSO {s:18s} n={h[0]:3d} mlp_top_verifies={h[1]:3d} rule_top_verifies={h[2]:3d}")
    print(f"LOSO total n={tot[0]} mlp={tot[1]} rule={tot[2]} (multi-candidate decisions with a known good action)")

    net, loss = fit(rows, a.hidden, a.epochs, a.seed, dev)
    print(f"final fit: {len(rows)} rows, hidden {a.hidden}, epochs {a.epochs}, train BCE {loss:.4f}")
    w1, b1 = net[0].weight.detach().cpu(), net[0].bias.detach().cpu()
    w2, b2 = net[2].weight.detach().cpu()[0], float(net[2].bias.detach().cpu()[0])
    with open(a.mlp, "w") as f:
        f.write(f"{w1.shape[1]} {w1.shape[0]}\n")
        for row in w1.tolist():
            f.write(" ".join(f"{v:.7g}" for v in row) + "\n")
        f.write(" ".join(f"{v:.7g}" for v in b1.tolist()) + "\n")
        f.write(" ".join(f"{v:.7g}" for v in w2.tolist()) + "\n")
        f.write(f"{b2:.7g}\n")
    print(f"mlp weights -> {a.mlp}")


if __name__ == "__main__":
    main()
