"""Learned diagnosis vs the deterministic HELIX rules, on identical observations.

The healer logs what it observes (snap.jsonl, 4 Hz and at every decision). This
script turns those observations into per-node feature vectors, trains a gradient
boosted classifier on DEV episodes, and scores it on held-out episodes:

  A  decision point: at the snapshot where HELIX made its first decision in a
     faulty episode, which node is the root and what fault is it? The model and
     the rules see the same observation, so this compares diagnosis quality only.
  B  false alarms: fraction of fault-free snapshots (before injection, all
     episodes) where the model flags any node at p >= 0.5. HELIX's equivalent is
     its unnecessary-decision count from score.py.
  C  detection time: on held-out episodes run with policy=none (the fault evolves
     untouched), seconds from injection until the model flags the root with the
     right class on 2 consecutive snapshots.

Optionally (--llm MODEL) asks a local Ollama model the decision-point question,
constrained by a JSON schema to the closed node and fault enums.

  python3 eval/heal/learned.py --train eval/heal/out/dev_nav --test eval/heal/out/heldout_manip [--llm qwen3:4b-instruct]
"""

import argparse
import json
import time
import urllib.request
from collections import Counter
from pathlib import Path

import numpy as np

CLASSES = ["none", "crash", "hang", "degraded", "endpoint_loss", "cpu_runaway", "memory_leak", "qos_mismatch"]
# crash_loop is a count of quick deaths, not something one snapshot can show; for
# per-snapshot classification it is a crash.
LABEL_MAP = {"crash_loop": "crash"}
ONSET_S = 0.3  # observations this long after injection are labelled with the fault


def out_feats(n: dict) -> tuple[float, float, float]:
    outs = n.get("outs") or []
    if not outs:
        return 0.0, 1.0, 0.0
    age = max(o["age"] * max(o["hz"], 1e-3) for o in outs)  # in expected periods
    rate = min(o["rate"] / o["hz"] if o["hz"] > 0 else 1.0 for o in outs)
    own = float(any(not o["own"] for o in outs))
    return min(age, 200.0), min(rate, 2.0), own


def node_features(n: dict, by: dict[str, dict]) -> list[float]:
    age, rate, own = out_feats(n)
    ups = [by[u] for u in n.get("ups", []) if u in by and by[u].get("up", 0) > 0]
    uf = [out_feats(u) for u in ups] or [(0.0, 1.0, 0.0)]
    others = [m for m in by.values() if m["name"] != n["name"]]
    return [
        float(n["alive"]), min(n["up"], 30.0), min(n["down"], 30.0), n["cpu"], n["rss"] - n["base"], n["slope"],
        float(n["qd"]), float(n["qos"]), age, rate, own,
        max(a for a, _, _ in uf), min(r for _, r, _ in uf), max(o for _, _, o in uf),
        float(any(not u["alive"] for u in ups)),
        max((m["cpu"] for m in others), default=0.0),
        float(sum(out_feats(m)[0] > 3 for m in others)),
        float(len(ups)),
    ]


FEATURES = ["alive", "up", "down", "cpu", "rss_growth", "rss_slope", "quick_deaths", "qos_incompatible",
            "out_age_periods", "out_rate_frac", "own_endpoint_missing", "up_max_age", "up_min_rate",
            "up_endpoint_missing", "up_dead", "others_max_cpu", "others_stale", "n_upstream"]


def load_episode(d: Path) -> dict | None:
    try:
        meta = json.loads((d / "meta.json").read_text())
        probe = json.loads((d / "probe.json").read_text())
        lines = (d / "events.jsonl").read_text().splitlines()
        snaps = [json.loads(x) for x in (d / "snap.jsonl").read_text().splitlines()]
    except (OSError, json.JSONDecodeError):
        return None
    t0 = json.loads(lines[0])["t0_wall"]
    t_inj = (probe["t_inject"] - t0) if probe["t_inject"] else None
    # A node that just died reads alive=0, up=0, down=0 on the tick it died, the same
    # as a backup that was never started; only the history tells them apart.
    seen: set[str] = set()
    for s in snaps:
        seen |= {n["name"] for n in s["nodes"] if n["alive"] or n["up"] > 0 or n["down"] > 0}
        s["started"] = set(seen)
    return dict(meta=meta, snaps=snaps, t_inj=t_inj)


def rows_for(ep: dict, until_first_decision: bool):
    """Yield (t_rel, node_name, features, label, snap) for each node in each usable snapshot."""
    meta, t_inj = ep["meta"], ep["t_inj"]
    root = meta["root"]
    label = LABEL_MAP.get(meta["expect"][0], meta["expect"][0]) if root else "none"
    first_dec = next((s["t"] for s in ep["snaps"] if s.get("decisions")), None)
    for s in ep["snaps"]:
        if until_first_decision and first_dec is not None and s["t"] > first_dec:
            break
        by = {n["name"]: n for n in s["nodes"]}
        t_rel = s["t"] - t_inj if t_inj is not None else -1e9
        for n in s["nodes"]:
            if n["name"] not in s["started"]:
                continue  # never started (an unused backup)
            if t_rel < 0:
                y = "none"
            elif n["name"] == root and t_rel >= ONSET_S:
                y = label
            elif n["name"] == root:
                continue  # onset window: ambiguous, not used for training or scoring
            else:
                y = "none"
            yield t_rel, n["name"], node_features(n, by), y, s


def load(dirs: list[str]) -> list[dict]:
    eps = []
    for root in dirs:
        for d in sorted(p for p in Path(root).iterdir() if p.is_dir()):
            ep = load_episode(d)
            if ep:
                eps.append(ep)
    return eps


def train(eps: list[dict]):
    from sklearn.ensemble import HistGradientBoostingClassifier
    X, y = [], []
    for ep in eps:
        # A repair changes what follows, so only policy=none episodes are used past the first decision.
        for _, _, f, lab, _ in rows_for(ep, until_first_decision=ep["meta"]["policy"] != "none"):
            X.append(f)
            y.append(CLASSES.index(lab))
    X, y = np.array(X), np.array(y)
    # Balance: the "none" class dominates by ~50x.
    cnt = Counter(y.tolist())
    w = np.array([len(y) / (len(cnt) * cnt[v]) for v in y])
    clf = HistGradientBoostingClassifier(max_iter=200, learning_rate=0.1, max_leaf_nodes=15, random_state=0)
    clf.fit(X, y, sample_weight=w)
    return clf, cnt


def pick(clf, snap: dict) -> tuple[str, str, float]:
    """Most likely (node, fault) in one snapshot; ("", "none") if no node is flagged."""
    by = {n["name"]: n for n in snap["nodes"]}
    names = [n["name"] for n in snap["nodes"] if n["name"] in snap["started"]]
    if not names:
        return "", "none", 0.0
    P = clf.predict_proba(np.array([node_features(by[k], by) for k in names]))
    cols = [j for j, k in enumerate(clf.classes_) if k != 0]  # columns follow clf.classes_; drop "none"
    if not cols:
        return "", "none", 0.0
    P = P[:, cols]
    i, c = np.unravel_index(np.argmax(P), P.shape)
    p = float(P[i, c])
    return (names[i], CLASSES[clf.classes_[cols[c]]], p) if p >= 0.5 else ("", "none", p)


def llm_pick(model: str, snap: dict) -> tuple[str, str, float]:
    names = [n["name"] for n in snap["nodes"] if n["name"] in snap["started"]]
    rows = []
    for n in snap["nodes"]:
        if n["name"] not in names:
            continue
        outs = ", ".join(f"{o['topic']} expected {o['hz']:.0f} Hz, measured {o['rate']:.1f} Hz, last msg {o['age']:.2f} s ago,"
                         f" {'still advertised' if o['own'] else 'NOT advertised'}" for o in n["outs"])
        rows.append(f"- {n['name']}: {'alive' if n['alive'] else 'DEAD for %.1f s' % n['down']}, up {n['up']:.1f} s, "
                    f"cpu {n['cpu']:.2f} cores, rss {n['rss']:.0f} MB (start {n['base']:.0f}, slope {n['slope']:.1f} MB/s), "
                    f"quick deaths {n['qd']}, qos incompatible {bool(n['qos'])}, inputs from {n['ups']}; outputs: {outs}")
    faults = CLASSES[1:]
    prompt = ("You diagnose a ROS 2 pipeline. Find the single ROOT-CAUSE node (not nodes that only starve because "
              "an upstream node failed) and its fault, or answer none if everything is healthy.\nFaults: crash (process "
              "dead), hang (alive, outputs stopped), degraded (alive, outputs at a fraction of the expected rate), "
              "endpoint_loss (alive but stopped advertising its output topic), cpu_runaway (sustained cpu >= 1.5 cores), "
              "memory_leak (rss growing steadily), qos_mismatch (a subscription cannot match its publisher).\n"
              "Nodes:\n" + "\n".join(rows) + "\nAnswer with JSON.")
    schema = {"type": "object", "properties": {"node": {"type": "string", "enum": names + ["none"]},
                                               "fault": {"type": "string", "enum": faults + ["none"]}},
              "required": ["node", "fault"]}
    body = json.dumps({"model": model, "prompt": prompt, "format": schema, "stream": False, "think": False,
                       "options": {"temperature": 0, "num_gpu": 99, "num_ctx": 4096}}).encode()
    t = time.time()
    req = urllib.request.Request("http://127.0.0.1:11434/api/generate", body, {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        ans = json.loads(json.loads(r.read())["response"])
    node, fault = ans.get("node", "none"), ans.get("fault", "none")
    if node not in names or fault not in faults:  # anything outside the enum is "no diagnosis"
        node, fault = "", "none"
    return node, fault, time.time() - t


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", nargs="+", required=True)
    ap.add_argument("--test", nargs="+", required=True)
    ap.add_argument("--llm", default="")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    tr, te = load(a.train), load(a.test)
    t = time.time()
    clf, cnt = train(tr)
    print(f"train: {len(tr)} episodes, {sum(cnt.values())} node-snapshots, classes "
          f"{ {CLASSES[k]: v for k, v in sorted(cnt.items())} }, fit {time.time() - t:.1f} s")
    res: dict = {}

    # A: decision point, helix episodes with a fault.
    hits = {"rules": 0, "gbdt": 0, "llm": 0}
    n = 0
    lat, rows = [], []
    for ep in te:
        m = ep["meta"]
        if m["policy"] != "helix" or not m["root"]:
            continue
        snap = next((s for s in ep["snaps"] if s.get("decisions")), None)
        if snap is None:
            continue
        n += 1
        roots = {m["root"]} | set(m.get("collateral", []))
        expect = {LABEL_MAP.get(e, e) for e in m["expect"]}
        d0 = snap["decisions"][0]
        rules_ok = d0["node"] == m["root"] and LABEL_MAP.get(d0["fault"], d0["fault"]) in expect
        gn, gf, gp = pick(clf, snap)
        g_ok = gn == m["root"] and gf in expect
        hits["rules"] += rules_ok
        hits["gbdt"] += g_ok
        row = dict(scenario=m["scenario"], rep=m["rep"], truth=f"{m['root']}:{'/'.join(expect)}",
                   rules=f"{d0['node']}:{d0['fault']}", gbdt=f"{gn}:{gf}:{gp:.2f}", rules_ok=rules_ok, gbdt_ok=g_ok,
                   gbdt_root_in_set=gn in roots)
        if a.llm:
            ln, lf, dt = llm_pick(a.llm, snap)
            hits["llm"] += ln == m["root"] and lf in expect
            lat.append(dt)
            row.update(llm=f"{ln}:{lf}", llm_ok=ln == m["root"] and lf in expect)
        rows.append(row)
    for r in rows:
        print(f"  {r['scenario']:<24}{r['rep']}  truth {r['truth']:<28} rules {r['rules']:<26} gbdt {r['gbdt']:<30}"
              + (f" llm {r['llm']}" if a.llm else ""))
    print(f"A decision point (n={n}): rules {hits['rules']}/{n}  gbdt {hits['gbdt']}/{n}"
          + (f"  llm[{a.llm}] {hits['llm']}/{n}, median {np.median(lat):.2f} s/call" if a.llm and lat else ""))
    res["A"] = dict(n=n, **hits, llm_median_s=float(np.median(lat)) if lat else None, rows=rows)

    # B: false alarms on fault-free observations.
    flags, total = 0, 0
    for ep in te:
        for s in ep["snaps"]:
            t_rel = s["t"] - ep["t_inj"] if ep["t_inj"] is not None else -1e9
            if ep["meta"]["root"] and t_rel >= 0:
                break
            if s["t"] < 3.0:
                continue  # startup grace: nothing is judged then
            total += 1
            flags += pick(clf, s)[0] != ""
    print(f"B false alarms: gbdt flags a node in {flags}/{total} fault-free snapshots ({100 * flags / max(total, 1):.2f}%)")
    res["B"] = dict(flags=flags, total=total)

    # C: detection time on untouched faults.
    det = {}
    for ep in te:
        m = ep["meta"]
        if m["policy"] != "none" or not m["root"]:
            continue
        expect = {LABEL_MAP.get(e, e) for e in m["expect"]}
        streak, found = 0, None
        for s in ep["snaps"]:
            t_rel = s["t"] - ep["t_inj"]
            if t_rel < 0:
                continue
            gn, gf, _ = pick(clf, s)
            streak = streak + 1 if (gn == m["root"] and gf in expect) else 0
            if streak >= 2:
                found = t_rel
                break
        det.setdefault(m["scenario"], []).append(found)
    for k, v in det.items():
        print(f"C {k:<24} gbdt detects root at {['%.2f' % x if x is not None else 'never' for x in v]} s")
    res["C"] = det
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1, default=str))


if __name__ == "__main__":
    main()
