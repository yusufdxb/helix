#!/usr/bin/env python3
"""HELIX v2 evaluation harness.

Replays recorded GO2 streams through the C++ monitor (monitor_eval), injects controlled
message-level faults, calibrates every threshold on DEV recordings only, and scores the
arms named in docs/preregistration/2026-11-02-helix-v2-content-faults-and-stop-verification.md.

  helix_eval.py calibrate            DEV only: writes eval/out/calibration.json and configs
  helix_eval.py score --split dev    pipeline check on DEV episodes (developmental)
  helix_eval.py score --split heldout  the single confirmatory pass (needs a clean git tree)

Inputs: one .npz per recording (eval/extract_bag.py) in $HELIX_NPZ.
"""
import argparse
import json
import math
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from scipy import stats

ROOT = Path(__file__).resolve().parents[1]
NPZ = Path(os.environ.get("HELIX_NPZ", ROOT / "eval/data"))
BIN = Path(os.environ.get("HELIX_MONITOR_EVAL", ROOT / "build/helix_monitor/monitor_eval"))
ZBIN = Path(os.environ.get("HELIX_DETECTOR_EVAL", ROOT / "build/helix_sensing_cpp/helix_detector_eval"))
OUT = ROOT / "eval/out"

# ------------------------------------------------------------------ registered split
DET_DEV = ["odin-hw-20260901-1926", "G1_varied_speed", "G1_yaw_in_place",
           "G1_negative_stationary", "G1_forward_only"]
DET_HO = ["odin-hw-standing-20260901-1935", "G2_prelim_asymmetric_drag", "G1_stride_segmentation"]
FA_DEV = ["motion", "motion2"]
FA_HO = ["motion3", "motion4", "fs_standing"]
STOP_DEV = ["faultwalk1"]
STOP_HO = ["motion4", "partialwalk1"]
NATIVE = ["20261017T222205_stop_motion", "20261017T222336_stop_motion", "20261017T222412_stop_motion",
          "20261017T223203_stop_motion", "20261017T223245_stop_motion"]
SPORT_SECONDARY = ["motion3", "motion4"]          # PHM held-out walks for sport-stream injection

ONSET_START, ONSET_STEP, ONSET_END_MARGIN = 10.0, 6.0, 5.0
W = 2.0                 # scoring window after onset
PRE, POST = 5.0, 2.5    # episode replay span around onset
CLASSES = ["F1", "F2", "F3a", "F3b", "F4_0.2", "F4_0.1", "F5", "F6", "F7", "BIAS_1.0"]
PRIMARY = ["F3a", "F4_0.2", "F5", "F6"]
ARMS = {
    "B-rate": {"rate"},
    "B-range": {"range"},
    "union": {"rate", "range"},
    "HELIX": {"rate", "frozen", "stamp", "residual"},
    "frozen": {"frozen"},
    "stamp": {"stamp"},
    "residual": {"residual"},
}
BZ_PARAMS = ["zscore_threshold=4.0", "window_size=60", "freeze_baseline=1", "std_floor_rel=0.02",
             "min_anomaly_duration_s=1.0", "duration_tolerance_s=0.25", "anchor_tolerance_rel=0.1"]
GYRO = [0, 1, 2]        # gyro channels in lidar_imu, sport and low payloads
SEED = 20261102


# ------------------------------------------------------------------ data
def fmt(name, t, stamp, v):
    vl = v.tolist()
    return [f"S,{name},{t[i]:.6f},{stamp[i]:.9f}," + ",".join(map(repr, vl[i])) for i in range(len(t))]


class Stream:
    def __init__(self, name, t, stamp, v):
        o = np.argsort(t, kind="stable")
        self.name, self.t, self.stamp, self.v = name, t[o], stamp[o], np.ascontiguousarray(v[o])
        self._lines = None

    def idx(self, lo, hi):
        return np.searchsorted(self.t, lo, "left"), np.searchsorted(self.t, hi, "left")

    def lines(self):
        if self._lines is None:
            self._lines = fmt(self.name, self.t, self.stamp, self.v)
        return self._lines

    def slice(self, lo, hi):
        i0, i1 = self.idx(lo, hi)
        return self.t[i0:i1].copy(), self.stamp[i0:i1].copy(), self.v[i0:i1].copy()

    def period(self):
        return float(np.median(np.diff(self.t)))


_CACHE = {}


def load(name):
    if name in _CACHE:
        return _CACHE[name]
    z = np.load(NPZ / f"{name}.npz")
    keys = [k for k in ("lidar_imu", "sport", "low") if f"{k}_t" in z.files]
    t0 = min(z[f"{k}_t"][0] for k in keys)
    S = {}
    if "lidar_imu" in keys:
        d = z["lidar_imu"]
        S["lidar_imu"] = Stream("lidar_imu", z["lidar_imu_t"] - t0, d[:, 0] - d[0, 0], d[:, 1:7])
    if "sport" in keys:
        d = z["sport"]   # payload: gx gy gz ax ay az | vx vy vz | yaw_speed
        S["sport"] = Stream("sport", z["sport_t"] - t0, d[:, 0] - d[0, 0], d[:, 1:11])
    if "low" in keys:
        d = z["low"]     # payload: gx gy gz ax ay az | dq0..dq11 ; tick is in ms
        S["low"] = Stream("low", z["low_t"] - t0, (d[:, 0] - d[0, 0]) * 1e-3, d[:, 1:19])
    stops = []
    if "req_t" in z.files:
        rt, ra = z["req_t"] - t0, z["req_api"]
        stops = [float(rt[i]) for i in range(1, len(ra)) if ra[i] == 1003 and ra[i - 1] == 1008]
        nxt = []
        for s in stops:
            later = rt[(ra == 1008) & (rt > s)]
            nxt.append(float(later[0] - s) if len(later) else math.inf)
    else:
        nxt = []
    dur = max(s.t[-1] for s in S.values())
    _CACHE[name] = (S, stops, nxt, dur)
    return _CACHE[name]


def episode_lines(S, lo, hi, override=None, extra=(), tick=0.02):
    ts, ls = [], []
    for name, s in S.items():
        if override and name in override:
            t, st, v = override[name]
            m = (t >= lo) & (t < hi)
            ls.extend(fmt(name, t[m], st[m], v[m]))
            ts.append(t[m])
        else:
            i0, i1 = s.idx(lo, hi)
            ls.extend(s.lines()[i0:i1])
            ts.append(s.t[i0:i1])
    ticks = np.arange(lo, hi, tick)
    ls.extend(f"T,{x:.6f}" for x in ticks)
    ts.append(ticks)
    for t, line in extra:
        ls.append(line)
        ts.append(np.array([t]))
    order = np.argsort(np.concatenate(ts), kind="stable")
    return [ls[i] for i in order]


# ------------------------------------------------------------------ fault injection
def inject(t, stamp, v, onset, cls, rng):
    """Return (t, stamp, v) with fault `cls` applied to samples received at or after onset."""
    t, stamp, v = t.copy(), stamp.copy(), v.copy()
    post = t >= onset
    if not post.any() or cls == "none":
        return t, stamp, v
    pre_idx = np.flatnonzero(~post)
    last = pre_idx[-1] if len(pre_idx) else np.flatnonzero(post)[0]
    if cls == "F1":
        keep = ~post
        return t[keep], stamp[keep], v[keep]
    if cls == "F2":
        k = np.cumsum(post) - 1
        keep = ~post | (k % 2 == 0)
        return t[keep], stamp[keep], v[keep]
    if cls == "F3a":
        v[post] = v[last]
        stamp[post] = stamp[last]
    elif cls == "F3b":
        v[post] = v[last]
    elif cls.startswith("F4_") or cls.startswith("BIAS_"):
        v[post, 2] += float(cls.split("_")[1])
    elif cls == "F5":
        v[np.ix_(post, GYRO)] *= 0.5
    elif cls == "F6":
        v[post, 2] *= -1.0
    elif cls == "F7":
        v[np.ix_(post, GYRO)] += rng.normal(0.0, 0.3, (int(post.sum()), 3))
    else:
        raise ValueError(cls)
    return t, stamp, v


def onsets(dur):
    return list(np.arange(ONSET_START, dur - ONSET_END_MARGIN + 1e-9, ONSET_STEP))


# ------------------------------------------------------------------ C++ runners
def run_eval(cfg, lines, extra_args=()):
    r = subprocess.run([str(BIN), str(cfg), *extra_args], input="\n".join(lines) + "\n",
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"monitor_eval failed: {r.stderr[:500]}")
    return [json.loads(x) for x in r.stdout.splitlines() if x]


def run_episodes(cfg, episodes):
    """episodes: list of (id, lines). Returns {id: [records]} with one fresh monitor per episode."""
    lines = []
    for eid, el in episodes:
        lines.append(f"R,{eid}")
        lines.extend(el)
    out, cur = {}, None
    for rec in run_eval(cfg, lines):
        if "episode" in rec:
            cur = rec["episode"]
            out[cur] = []
        elif cur is not None:
            out[cur].append(rec)
    return out


def pmap(fn, items, workers=8):
    with ThreadPoolExecutor(workers) as ex:
        return list(ex.map(fn, items))


# ------------------------------------------------------------------ configs
def config_text(cal, streams, kinds, residual_key="residual"):
    L = [f"# generated by eval/helix_eval.py from calibration.json ({cal['source']})"]
    for s in streams:
        c = cal["streams"][s]
        if "rate" in kinds:
            r = c["rate"]
            L.append(f"rate {s} window_s=1.0 min_hz={r['min_hz']:.6g} max_hz={r['max_hz']:.6g} "
                     f"timeout_s={r['timeout_s']:.6g} clear_s=0.5")
        if "frozen" in kinds:
            L.append(f"frozen {s} frozen_s={c['frozen_s']:.6g} min_count=3")
        if "stamp" in kinds:
            L.append(f"stamp {s} stall_s={c['stall_s']:.6g}")
        if "range" in kinds:
            n = len(c["lo"])
            L.append(f"range {s} ch={','.join(map(str, range(n)))} lo={','.join(f'{x:.9g}' for x in c['lo'])} "
                     f"hi={','.join(f'{x:.9g}' for x in c['hi'])} hold_s=0 clear_s=0.5")
    if "residual" in kinds and "low" in streams and "lidar_imu" in streams and residual_key in cal:
        L.append(residual_line(cal[residual_key]))
    return "\n".join(L) + "\n"


def residual_line(rc, threshold=None):
    thr = rc["threshold"] if threshold is None else threshold
    return (f"residual low lidar_imu a_ch=0,1,2 b_ch=0,1,2 R={','.join(f'{x:.9g}' for x in rc['R'])} "
            f"d={','.join(f'{x:.9g}' for x in rc['d'])} tau_s={rc['tau_s']} threshold={thr:.9g} "
            f"hold_s={rc['hold_s']} clear_s=0.5 stale_s=0.1")


def stop_text(cal, arm):
    sc = cal["stop"]
    if arm == "B-est":
        g = sc["estimator"]
        return (f"group estimator sport ch=6,7,9 scale=20,20,10 raw=0 max=1 quiet=1.0 moving=1e9\n"
                f"stop window_s=0.5 deadline_s=2.0 stale_s=0.2 min_groups=1 require_raw=0\n")
    e, j, i = sc["estimator"], sc["joints"], sc["inertial"]
    return (f"group estimator sport ch=6,7,9 scale=20,20,10 raw=0 max=1 quiet={e['quiet']:.6g} moving={e['moving']:.6g}\n"
            f"group joints low ch={','.join(str(c) for c in range(6, 18))} raw=1 quiet={j['quiet']:.6g} moving={j['moving']:.6g}\n"
            f"group inertial low ch=0,1,2 raw=1 quiet={i['quiet']:.6g} moving={i['moving']:.6g}\n"
            f"stop window_s=0.5 deadline_s=2.0 stale_s=0.2 min_groups=2 require_raw=1\n")


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


# ------------------------------------------------------------------ calibration helpers
def lowpass(t, x, tau):
    """Same recursion as ResidualCheck::lp_update."""
    y = np.empty_like(x)
    y[0] = x[0]
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        if dt > 10 * tau:
            y[k] = x[k]
        else:
            y[k] = y[k - 1] + (1 - math.exp(-dt / tau)) * (x[k] - y[k - 1])
    return y


def lp_on_grid(s, grid, tau):
    y = lowpass(s.t, s.v[:, GYRO], tau)
    return np.stack([np.interp(grid, s.t, y[:, i]) for i in range(3)], 1)


def kabsch(A, B):
    """R, d minimizing ||A - (R B + d)||: rows are samples."""
    ma, mb = A.mean(0), B.mean(0)
    H = (B - mb).T @ (A - ma)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, ma - R @ mb


def shifted(S, seconds=20.0):
    """Sham input: LiDAR payload circularly shifted by `seconds` (timing kept, alignment removed)."""
    s = S["lidar_imu"]
    k = int(round(seconds / s.period()))
    out = dict(S)
    out["lidar_imu"] = Stream("lidar_imu", s.t, s.stamp, np.roll(s.v, -k, axis=0))
    return out


def fit_residual(recs, tau=0.1, sham=False):
    A, B, lags = [], [], []
    for name in recs:
        S = load(name)[0]
        if sham:
            S = shifted(S)
        lo, hi = S["low"], S["lidar_imu"]
        grid = np.arange(max(lo.t[0], hi.t[0]) + 1, min(lo.t[-1], hi.t[-1]) - 1, 0.01)
        a, b = lp_on_grid(lo, grid, tau), lp_on_grid(hi, grid, tau)
        A.append(a)
        B.append(b)
        # inter-IMU lag on the raw (unfiltered) gyro norm, +-100 ms
        ga = np.linalg.norm(np.stack([np.interp(grid, lo.t, lo.v[:, i]) for i in GYRO], 1), axis=1)
        gb = np.linalg.norm(np.stack([np.interp(grid, hi.t, hi.v[:, i]) for i in GYRO], 1), axis=1)
        ga, gb = ga - ga.mean(), gb - gb.mean()
        best = max(range(-10, 11), key=lambda k: float(np.dot(ga[10:-10], np.roll(gb, k)[10:-10])))
        lags.append(best * 0.01)
    R, d = kabsch(np.concatenate(A), np.concatenate(B))
    return {"R": R.flatten().tolist(), "d": d.tolist(), "tau_s": tau, "hold_s": 0.2, "lag_s": lags}


# ------------------------------------------------------------------ calibrate
def calibrate(args):
    cal = {"source": "DEV only: " + ", ".join(DET_DEV + FA_DEV + STOP_DEV), "streams": {}}
    permissive = ""
    for s in ("lidar_imu", "sport", "low"):
        permissive += f"rate {s} window_s=1.0 min_hz=0 max_hz=1e9 timeout_s=1e9 clear_s=0.5\n"
        permissive += f"frozen {s} frozen_s=1e9 min_count=3\nstamp {s} stall_s=1e9\n"
    pcfg = write(OUT / "cfg/permissive.cfg", permissive)

    def stats_of(name):
        S = load(name)[0]
        lo, hi = 0.0, max(s.t[-1] for s in S.values()) + 0.1
        return name, run_eval(pcfg, episode_lines(S, lo, hi), ["--stats"])

    agg = {}
    for name, recs in pmap(stats_of, DET_DEV + FA_DEV):
        S = load(name)[0]
        for r in recs:
            if r.get("stat") is None or r["stream"] not in S:
                continue
            a = agg.setdefault(r["stream"], {"min_rate": [], "max_rate": [], "max_gap": [], "frozen": [], "stamp": []})
            if r["stat"] == "rate":
                a["min_rate"].append(r["min_rate"])
                a["max_rate"].append(r["max_rate"])
                a["max_gap"].append(r["max_gap"])
            else:
                a[r["stat"]].append(r["max"])
    for s, a in agg.items():
        recs = [n for n in DET_DEV + FA_DEV if s in load(n)[0]]
        allv = np.concatenate([load(n)[0][s].v for n in recs])
        vmin, vmax = allv.min(0), allv.max(0)
        span = vmax - vmin
        period = float(np.median([load(n)[0][s].period() for n in recs]))
        cal["streams"][s] = {
            "nominal_period_s": period,
            "dev": {k: (float(min(v)) if k == "min_rate" else float(max(v))) for k, v in a.items()},
            "rate": {"min_hz": 0.8 * min(a["min_rate"]), "max_hz": 1.2 * max(a["max_rate"]),
                     "timeout_s": max(3 * period, 1.5 * max(a["max_gap"]))},
            "frozen_s": max(0.05, 1.5 * max(a["frozen"])),
            "stall_s": max(0.05, 1.5 * max(a["stamp"])),
            "lo": (vmin - 0.1 * span).tolist(),
            "hi": (vmax + 0.1 * span).tolist(),
            "recordings": recs,
        }

    # residual: rotation and offset from DEV, threshold = 1.5 x DEV max held residual
    for key, sham in (("residual", False), ("residual_sham", True)):
        rc = fit_residual(DET_DEV, sham=sham)
        rcfg = write(OUT / f"cfg/{key}_fit.cfg", residual_line(rc, threshold=1e9))

        def held(name):
            S = load(name)[0]
            S = shifted(S) if sham else S
            S = {k: S[k] for k in ("low", "lidar_imu")}
            recs = run_eval(rcfg, episode_lines(S, 0.0, max(s.t[-1] for s in S.values()) + 0.1), ["--stats"])
            return [r["max"] for r in recs if r.get("stat") == "residual"][0]

        hm = pmap(held, DET_DEV)
        rc["dev_held_max"] = dict(zip(DET_DEV, hm))
        rc["threshold"] = 1.5 * max(hm)
        cal[key] = rc

    # stop verifier thresholds from faultwalk1 only
    tcfg = write(OUT / "cfg/stop_trace.cfg",
                 "group estimator sport ch=6,7,9 scale=20,20,10 raw=0 max=1\n"
                 f"group joints low ch={','.join(str(c) for c in range(6, 18))} raw=1\n"
                 "group inertial low ch=0,1,2 raw=1\nstop\n")
    S, stops, _, dur = load(STOP_DEV[0])
    tr = [r for r in run_eval(tcfg, episode_lines(S, 0.0, dur + 0.1), ["--trace", "0.05"]) if "trace" in r]
    tt = np.array([r["trace"] for r in tr])
    sc = {}
    for g in ("estimator", "joints", "inertial"):
        m = np.array([np.nan if r["metrics"][g] is None else r["metrics"][g] for r in tr], dtype=float)
        post = np.zeros_like(tt, bool)
        pre = np.zeros_like(tt, bool)
        for s0 in stops:
            post |= (tt >= s0 + 1.0) & (tt <= s0 + 2.0)
            pre |= (tt >= s0 - 4.0) & (tt <= s0 - 0.5)
        q = 1.5 * np.nanpercentile(m[post], 95)
        mv = 0.5 * np.nanpercentile(m[pre], 5)
        if g == "estimator":
            q = 1.0      # B-est definition: 0.05 m/s and 0.1 rad/s
        gm = None
        if q >= mv:
            gm = math.sqrt(q * mv)
            q = mv = gm
        sc[g] = {"quiet": float(q), "moving": float(mv), "post_p95": float(np.nanpercentile(m[post], 95)),
                 "pre_p5": float(np.nanpercentile(m[pre], 5)), "geo_mean_used": gm is not None,
                 "n_post": int(np.isfinite(m[post]).sum()), "n_pre": int(np.isfinite(m[pre]).sum())}
    cal["stop"] = sc
    write(OUT / "calibration.json", json.dumps(cal, indent=1))
    emit_configs(cal)
    print(json.dumps({k: cal[k] for k in ("residual", "residual_sham", "stop")}, indent=1)[:4000])
    for s, c in cal["streams"].items():
        print(s, "dev", {k: round(v, 4) for k, v in c["dev"].items()}, "rate", {k: round(v, 4) for k, v in c["rate"].items()},
              "frozen_s %.4f stall_s %.4f" % (c["frozen_s"], c["stall_s"]))


def emit_configs(cal):
    allk = {"rate", "frozen", "stamp", "range", "residual"}
    write(OUT / "cfg/monitor_full.cfg", config_text(cal, ["lidar_imu", "sport", "low"], allk))
    write(OUT / "cfg/monitor_body.cfg", config_text(cal, ["sport", "low"], allk))
    write(OUT / "cfg/sham.cfg", residual_line(cal["residual_sham"]) + "\n")
    write(OUT / "cfg/stop_helix.cfg", stop_text(cal, "HELIX"))
    write(OUT / "cfg/stop_best.cfg", stop_text(cal, "B-est"))
    # The ROS node config ships the same thresholds.
    write(ROOT / "src/helix_monitor/config/go2.cfg",
          config_text(cal, ["lidar_imu", "sport", "low"], {"rate", "frozen", "stamp", "residual"})
          + stop_text(cal, "HELIX"))


# ------------------------------------------------------------------ statistics
def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"),) * 3
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, max(0.0, c - h), min(1.0, c + h)


def poisson_upper(k, hours):
    return stats.chi2.ppf(0.975, 2 * (k + 1)) / 2 / hours


def block_bootstrap_diff(eps, a, b, n=10000, block=30.0, seed=SEED):
    """Pooled detection difference a - b; resample 30 s blocks within each recording."""
    rng = np.random.default_rng(seed)
    recs = {}
    for e in eps:
        recs.setdefault(e["rec"], {}).setdefault(int(e["onset"] // block), []).append(
            float(e["det"][a] is not None) - float(e["det"][b] is not None))
    tot_n = np.zeros(n)
    tot_s = np.zeros(n)
    for blocks in recs.values():
        bl = list(blocks.values())
        bn = np.array([len(x) for x in bl])
        bs = np.array([sum(x) for x in bl])
        idx = rng.integers(0, len(bl), (n, len(bl)))
        tot_n += bn[idx].sum(1)
        tot_s += bs[idx].sum(1)
    d = tot_s / tot_n
    return float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))


# ------------------------------------------------------------------ detection study
def first_raise(records, checks, stream, lo, hi):
    best = None
    for r in records:
        if r.get("raised") and r["check"] in checks and stream in r["stream"].split("|") and lo <= r["t"] <= hi:
            best = r["t"] if best is None else min(best, r["t"])
    return None if best is None else best - lo


def detection_episodes(recs, cfg, inj_stream, classes, sham=False, arms=ARMS):
    jobs = [(rec, cls) for rec in recs for cls in classes]

    def job(rc):
        rec, cls = rc
        S, _, _, dur = load(rec)
        if sham:
            S = shifted(S)
            S = {k: S[k] for k in ("low", "lidar_imu")}
        rng = np.random.default_rng([SEED, CLASSES.index(cls), DET_HO.index(rec) if rec in DET_HO else 99])
        eps = []
        for k, on in enumerate(onsets(dur)):
            lo, hi = on - PRE, on + POST
            ov = {inj_stream: inject(*S[inj_stream].slice(lo, hi), on, cls, rng)}
            eps.append((f"{rec}|{cls}|{k}", episode_lines(S, lo, hi, override=ov)))
        res = run_episodes(cfg, eps)
        out = []
        for k, on in enumerate(onsets(dur)):
            r = res.get(f"{rec}|{cls}|{k}", [])
            out.append({"rec": rec, "cls": cls, "onset": float(on),
                        "det": {a: first_raise(r, c, inj_stream, on, on + W) for a, c in arms.items()}})
        return out

    return [e for part in pmap(job, jobs) for e in part]


def clean_alarms(recs, cfg_for, sham=False):
    out = {}

    def job(rec):
        S, _, _, dur = load(rec)
        if sham:
            S = shifted(S)
            S = {k: S[k] for k in ("low", "lidar_imu")}
        end = min(s.t[-1] for s in S.values()) + 1e-6   # replay stops at the first stream's last message
        recs_ = run_eval(cfg_for(S), episode_lines(S, 0.0, end), ["--stats"])
        return rec, end, [r for r in recs_ if r.get("raised")], [r for r in recs_ if "stat" in r]

    for rec, dur, alarms, st in pmap(job, recs):
        out[rec] = {"hours": dur / 3600.0, "alarms": alarms, "stats": st}
    return out


def fa_table(clean, arms):
    tab = {}
    hours = sum(c["hours"] for c in clean.values())
    for a, checks in arms.items():
        k = sum(1 for c in clean.values() for r in c["alarms"] if r["check"] in checks)
        early = sum(1 for c in clean.values() for r in c["alarms"] if r["check"] in checks and r["t"] < 1.0)
        tab[a] = {"alarms": k, "hours": hours, "fa_per_h": k / hours, "fa_upper95": poisson_upper(k, hours),
                  "frac_first_1s": (early / k) if k else 0.0}
    return tab


def bz_rate_metric(t_msgs, t_end, period=0.5, window=5.0):
    ticks = np.arange(period, t_end, period)
    i0 = np.searchsorted(t_msgs, ticks - window, "left")
    i1 = np.searchsorted(t_msgs, ticks, "right")
    n = i1 - i0
    out = []
    for tk, a, b, c in zip(ticks, i0, i1, n):
        if c == 0:
            out.append((tk, "nan"))
        elif c == 1:
            out.append((tk, "0"))
        else:
            span = t_msgs[b - 1] - t_msgs[a]
            out.append((tk, repr(float((c - 1) / span)) if span >= 1e-6 else "0"))
    return out


def run_bz(lines):
    r = subprocess.run([str(ZBIN), *BZ_PARAMS], input="\n".join(f"{t:.6f},lidar_imu_rate,{v}" for t, v in lines) + "\n",
                       capture_output=True, text=True, check=True)
    return [json.loads(x) for x in r.stdout.splitlines() if x]


def bz_study(recs):
    eps, fa = [], {"alarms": 0, "hours": 0.0}
    for rec in recs:
        S, _, _, dur = load(rec)
        s = S["lidar_imu"]
        ev = run_bz(bz_rate_metric(s.t, dur))
        fa["alarms"] += sum(1 for e in ev if e["kind"] == "fault")
        fa["hours"] += dur / 3600.0
        for cls in ("F1", "F2"):
            for on in onsets(dur):
                t, _, _ = inject(s.t, s.stamp, s.v, on, cls, None)
                ev = run_bz(bz_rate_metric(t, on + POST))
                hit = [e["t"] - on for e in ev if e["kind"] == "fault" and on <= e["t"] <= on + W]
                eps.append({"rec": rec, "cls": cls, "onset": float(on), "det": min(hit) if hit else None})
    fa["fa_per_h"] = fa["alarms"] / fa["hours"]
    fa["fa_upper95"] = poisson_upper(fa["alarms"], fa["hours"])
    return eps, fa


def summarize(eps, arms, classes):
    tab = {}
    for cls in classes:
        sub = [e for e in eps if e["cls"] == cls]
        row = {}
        for a in arms:
            lat = [e["det"][a] for e in sub if e["det"][a] is not None]
            p, lo, hi = wilson(len(lat), len(sub))
            row[a] = {"k": len(lat), "n": len(sub), "rate": p, "ci": [lo, hi],
                      "lat_med": float(np.median(lat)) if lat else None,
                      "lat_p95": float(np.percentile(lat, 95)) if lat else None}
        tab[cls] = row
    return tab


# ------------------------------------------------------------------ stop study
def stop_variants(S, s0, variant, standing=None):
    """Return override dict for a stop at s0. Splice: post-stop streams are the 2.5 s before."""
    if variant == "genuine":
        return None
    ov = {}
    for name in ("sport", "low"):
        if name not in S:
            continue
        s = S[name]
        tpre, stpre, vpre = s.slice(s0 - POST, s0)
        tkeep, stkeep, vkeep = s.slice(s0 - 3.5, s0)
        t2, st2, v2 = tpre + POST, stpre + POST, vpre.copy()
        if variant in ("lies", "sham") and name == "sport":
            v2[:, 6:10] = 0.0
        if variant == "sham" and name == "low":
            ts, sts, vs = standing
            t2, st2, v2 = ts, sts, vs
        ov[name] = (np.concatenate([tkeep, t2]), np.concatenate([stkeep, st2]), np.concatenate([vkeep, v2]))
    return ov


def stop_study(recs, cfgs, variants, seed=SEED):
    rng = np.random.default_rng(seed)
    pool = [(r, s0) for r in recs for s0, g in zip(load(r)[1], load(r)[2]) if g >= 4.0]
    rows = []
    for rec in recs:
        S, stops, _, _ = load(rec)
        S = {k: v for k, v in S.items() if k in ("sport", "low")}
        for k, s0 in enumerate(stops):
            choices = [p for p in pool if p != (rec, s0)]
            pr, ps = choices[rng.integers(len(choices))] if choices else (rec, s0)
            standing = None
            if "low" in S and pr:
                lo = load(pr)[0]["low"]
                ts, sts, vs = lo.slice(ps + 1.0, ps + 1.0 + POST)
                standing = (ts - ps - 1.0 + s0, sts - ps - 1.0 + s0, vs)
            for var in variants:
                ov = stop_variants(S, s0, var, standing)
                lines = episode_lines(S, s0 - 3.0, s0 + POST, override=ov, extra=[(s0, f"C,stop,{s0:.6f}")])
                row = {"rec": rec, "event": k, "t_cmd": s0, "variant": var, "sham_source": f"{pr}@{ps:.2f}"}
                for arm, cfg in cfgs.items():
                    out = [r for r in run_eval(cfg, lines) if "stop" in r]
                    row[arm] = out[0]["stop"] if out else "NONE"
                    if arm == "HELIX" and out:
                        row["helix_metrics"] = out[0]["metrics"]
                        row["helix_latency"] = out[0]["t"] - s0
                row["B-ack"] = "VERIFIED"
                rows.append(row)
    return rows


def stop_summary(rows):
    tab = {}
    for var in sorted({r["variant"] for r in rows}):
        sub = [r for r in rows if r["variant"] == var]
        tab[var] = {}
        for arm in ("HELIX", "B-est", "B-ack"):
            counts = {}
            for r in sub:
                counts[r[arm]] = counts.get(r[arm], 0) + 1
            kv = counts.get("VERIFIED", 0)
            kf = counts.get("FAILED", 0)
            tab[var][arm] = {"n": len(sub), "counts": counts, "verified": wilson(kv, len(sub)),
                             "failed": wilson(kf, len(sub)),
                             "not_verified": wilson(len(sub) - kv, len(sub))}
    return tab


# ------------------------------------------------------------------ score
def git_state():
    h = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT,
                           capture_output=True, text=True).stdout.strip()
    return h, bool(dirty)


def score(args):
    cal = json.loads((OUT / "calibration.json").read_text())
    emit_configs(cal)
    h, dirty = git_state()
    if args.split == "heldout" and dirty and not args.allow_dirty:
        sys.exit("refusing the confirmatory pass on a dirty tree: commit the frozen code first")
    det, fa_recs, stop_recs, sport_recs = ((DET_DEV, FA_DEV, STOP_DEV, FA_DEV) if args.split == "dev"
                                           else (DET_HO, FA_HO, STOP_HO, SPORT_SECONDARY))
    full, body = OUT / "cfg/monitor_full.cfg", OUT / "cfg/monitor_body.cfg"
    res = {"split": args.split, "commit": h, "dirty": dirty}

    # C1 detection
    eps = detection_episodes(det, full, "lidar_imu", CLASSES)
    res["detection"] = summarize(eps, ARMS, CLASSES)
    prim = [e for e in eps if e["cls"] in PRIMARY]
    d = np.mean([(e["det"]["HELIX"] is not None) - (e["det"]["union"] is not None) for e in prim])
    res["primary_diff"] = {"diff": float(d), "ci95": block_bootstrap_diff(prim, "HELIX", "union"),
                           "per_recording": {r: float(np.mean([(e["det"]["HELIX"] is not None) - (e["det"]["union"] is not None)
                                                               for e in prim if e["rec"] == r])) for r in det
                                             if any(e["rec"] == r for e in prim)}}
    # sham and oracle on the residual family
    sham_eps = detection_episodes(det, OUT / "cfg/sham.cfg", "lidar_imu", CLASSES, sham=True,
                                  arms={"sham": {"residual"}})
    res["sham_detection"] = summarize(sham_eps, {"sham": 1}, CLASSES)

    clean = clean_alarms(det, lambda S: full)
    clean.update(clean_alarms(fa_recs, lambda S: body))
    res["clean_fa"] = fa_table(clean, ARMS)
    res["clean_fa_by_rec"] = {r: {a: sum(1 for x in c["alarms"] if x["check"] in ch) for a, ch in ARMS.items()}
                              | {"hours": c["hours"]} for r, c in clean.items()}
    res["clean_alarm_examples"] = {r: c["alarms"][:20] for r, c in clean.items()}
    sham_clean = clean_alarms(det, lambda S: OUT / "cfg/sham.cfg", sham=True)
    res["sham_fa"] = fa_table(sham_clean, {"sham": {"residual"}})

    held = [x["max"] for c in clean.values() for x in c["stats"] if x.get("stat") == "residual"]
    ocfg = write(OUT / f"cfg/oracle_{args.split}.cfg", residual_line(cal["residual"], threshold=max(held) * 1.001) + "\n")
    or_eps = detection_episodes(det, ocfg, "lidar_imu", CLASSES, arms={"oracle": {"residual"}})
    res["oracle"] = {"threshold": max(held) * 1.001, "detection": summarize(or_eps, {"oracle": 1}, CLASSES)}

    # secondary: sport-stream dropout and frozen on PHM walks
    sp = detection_episodes(sport_recs, body, "sport", ["F1", "F3a"])
    res["sport_secondary"] = summarize(sp, ARMS, ["F1", "F3a"])

    # K4: B-z versus B-rate on F1, F2
    bz_eps, bz_fa = bz_study(det)
    res["bz"] = {"detection": {c: wilson(sum(e["det"] is not None for e in bz_eps if e["cls"] == c),
                                         sum(e["cls"] == c for e in bz_eps)) for c in ("F1", "F2")},
                 "clean_fa_lidar_rate": bz_fa,
                 "brate_clean_fa_lidar": sum(1 for c in clean.values() for r in c["alarms"]
                                             if r["check"] == "rate" and r["stream"] == "lidar_imu")}

    # C2 stop verification
    cfgs = {"HELIX": OUT / "cfg/stop_helix.cfg", "B-est": OUT / "cfg/stop_best.cfg"}
    rows = stop_study(stop_recs, cfgs, ["genuine", "splice", "lies", "sham"])
    res["stop"] = stop_summary(rows)
    nat = stop_study(NATIVE, cfgs, ["genuine"])
    res["stop_native"] = stop_summary(nat) if nat else {}

    # positive controls
    det_tab = res["detection"]
    res["positive_controls"] = {
        "F1_B-rate": det_tab["F1"]["B-rate"]["rate"], "F1_HELIX": det_tab["F1"]["HELIX"]["rate"],
        "BIAS_1.0_HELIX": det_tab["BIAS_1.0"]["HELIX"]["rate"],
        "splice_HELIX_failed": res["stop"]["splice"]["HELIX"]["failed"][0],
        "splice_B-est_not_verified": res["stop"]["splice"]["B-est"]["not_verified"][0],
    }
    res["positive_controls_pass"] = all(v >= 0.95 for v in res["positive_controls"].values())

    tag = args.split
    write(OUT / f"results_{tag}.json", json.dumps(res, indent=1, default=float))
    with open(OUT / f"episodes_{tag}.jsonl", "w") as f:
        for e in eps + sham_eps + or_eps + sp:
            f.write(json.dumps(e) + "\n")
        for e in bz_eps:
            f.write(json.dumps({"bz": 1, **e}) + "\n")
        for r in rows + nat:
            f.write(json.dumps({"stop": 1, **r}) + "\n")
    report(res)


def report(res):
    print(f"== split {res['split']}  commit {res['commit'][:7]}  dirty={res['dirty']}")
    print("detection rate within 2.0 s (k/n):")
    arms = ["B-rate", "B-range", "union", "HELIX", "frozen", "stamp", "residual"]
    print("  %-9s" % "class" + "".join("%11s" % a for a in arms) + "%9s%9s" % ("sham", "oracle"))
    for cls, row in res["detection"].items():
        sh = res["sham_detection"][cls]["sham"]
        orc = res["oracle"]["detection"][cls]["oracle"]
        print("  %-9s" % cls + "".join("%7d/%-3d" % (row[a]["k"], row[a]["n"]) for a in arms)
              + "%5d/%-3d%5d/%-3d" % (sh["k"], sh["n"], orc["k"], orc["n"]))
    pdiff = res["primary_diff"]
    print("primary pooled HELIX - union: %.3f  CI95 [%.3f, %.3f]  per-rec %s" % (
        pdiff["diff"], *pdiff["ci95"], {k: round(v, 3) for k, v in pdiff["per_recording"].items()}))
    print("clean FA/h (upper95):", {a: "%d/%.3fh=%.2f (%.2f)" % (v["alarms"], v["hours"], v["fa_per_h"], v["fa_upper95"])
                                    for a, v in res["clean_fa"].items()})
    print("sham FA:", res["sham_fa"]["sham"]["alarms"], " oracle thr %.4f" % res["oracle"]["threshold"])
    print("B-z:", {c: "%.3f" % v[0] for c, v in res["bz"]["detection"].items()}, "B-z clean FA",
          res["bz"]["clean_fa_lidar_rate"]["alarms"], " B-rate lidar clean FA", res["bz"]["brate_clean_fa_lidar"])
    print("sport secondary:", {c: {a: row[a]["k"] for a in ("B-rate", "B-range", "HELIX")} | {"n": row["HELIX"]["n"]}
                               for c, row in res["sport_secondary"].items()})
    for var, row in res["stop"].items():
        print("stop %-8s" % var, {a: row[a]["counts"] for a in row})
    for var, row in res["stop_native"].items():
        print("native %-6s" % var, {a: row[a]["counts"] for a in row})
    print("positive controls:", {k: round(v, 3) for k, v in res["positive_controls"].items()},
          "PASS" if res["positive_controls_pass"] else "FAIL")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("calibrate")
    sp = sub.add_parser("score")
    sp.add_argument("--split", choices=["dev", "heldout"], required=True)
    sp.add_argument("--allow-dirty", action="store_true")
    a = ap.parse_args()
    {"calibrate": calibrate, "score": score}[a.cmd](a)


if __name__ == "__main__":
    main()
