"""Run controlled fault-injection episodes against helix_healer.

One episode = one healer process under one policy, supervising the spec's
pipeline in its own ROS domain on its own CPU slice, with one fault injected at
a fixed time by an independent probe. Episodes run in parallel slots.

  python3 eval/heal/run_episodes.py --set nav --policies none,respawn,watchdog,helix \
      --reps 3 --out eval/heal/out/dev

The humanoid set is different: the stack is stock ROS 2 software generated per episode by
humanoid/make_stack.py, and faults act on the real processes from a thread in this runner
(humanoid/inject.py), so the probe only observes.
"""

import argparse
import itertools
import json
import os
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scenarios import SETS  # noqa: E402
from humanoid.inject import Injector  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
HEALER = ROOT / "install/helix_healer/lib/helix_healer/helix_healer_node"
PROBE = Path(__file__).resolve().parent / "probe.py"
SPECS = {"nav": ROOT / "src/helix_healer/config/nav.spec", "manip": ROOT / "src/helix_healer/config/manip.spec"}
MAKE_STACK = Path(__file__).resolve().parent / "humanoid/make_stack.py"
PROBE_CORES = "20-23"
TYPES = {"/joint_states": "sensor_msgs/msg/JointState", "/tf": "tf2_msgs/msg/TFMessage", "/image": "sensor_msgs/msg/Image"}
# Same policy as `helix`, with the v3 release settings: 20 Hz tick, backoff before the first
# restart, fixed staleness rule. Isolates what the latency work bought.
V3_PARAMS = "healer tick_hz=20 fast_repair=0 adaptive=0\n"


def spec_outputs(text: str) -> dict[str, list[tuple[str, float]]]:
    out: dict[str, list[tuple[str, float]]] = {}
    for line in text.splitlines():
        tok = line.split("#", 1)[0].split()
        if len(tok) >= 3 and tok[0] == "out":
            out.setdefault(tok[1], []).append((tok[2], float(tok[3]) if len(tok) > 3 else 0.0))
    return out


def with_fault_files(text: str, ff: Path) -> str:
    lines = []
    for line in text.splitlines():
        tok = line.split()
        if tok and tok[0] == "exec":
            line = f"{line} -p fault_file:={ff / tok[1]}"
        lines.append(line)
    return "\n".join(lines) + "\n"


def proc_cpu_s(pid: int) -> float:
    try:
        f = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return (int(f[11]) + int(f[12])) / os.sysconf("SC_CLK_TCK")
    except (OSError, IndexError):
        return float("nan")


def proc_rss_mb(pid: int) -> float:
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024
    except OSError:
        pass
    return float("nan")


def reap(tag: str) -> int:
    """Kill whatever this episode left running (found by its HELIX_EPISODE tag) and count it."""
    me, n = os.getpid(), 0
    needle = f"HELIX_EPISODE={tag}".encode()
    for p in os.listdir("/proc"):
        if not p.isdigit() or int(p) == me:
            continue
        try:
            if needle in Path(f"/proc/{p}/environ").read_bytes().split(b"\0"):
                os.kill(int(p), signal.SIGCONT)
                os.kill(int(p), signal.SIGKILL)
                n += 1
        except OSError:
            pass
    return n


def run_episode(ep: dict, slot: int, warm: float, post: float) -> dict:
    d = Path(ep["dir"])
    (d / "ff").mkdir(parents=True, exist_ok=True)
    (d / "logs").mkdir(exist_ok=True)
    real = ep["set"] == "humanoid"
    if real:
        subprocess.run([sys.executable, str(MAKE_STACK), str(d / "stack")], check=True, capture_output=True)
        text = (d / "stack/humanoid.spec").read_text()
    else:
        text = with_fault_files(Path(SPECS[ep["set"]]).read_text(), d / "ff")
    policy = ep["policy"]
    if policy == "helix_v3params":
        policy, text = "helix", text + V3_PARAMS
    (d / "spec").write_text(text)
    env = dict(os.environ, ROS_DOMAIN_ID=str(30 + slot), RMW_IMPLEMENTATION="rmw_cyclonedds_cpp", HELIX_EPISODE=str(d),
               ROS_LOCALHOST_ONLY="1")
    cores = f"{4 * slot}-{4 * slot + 3}"
    t_start = time.time()
    t_inject, t_end = t_start + warm, t_start + warm + post
    topics = sorted({t for outs in spec_outputs(text).values() for t, _ in outs})
    if real:
        topics = [f"{t}={TYPES[t]}" for t in topics]
    inj = Injector(ep["inject"] if real else "", str(30 + slot), env, t_inject, t_end, d / "logs",
                   seed=f"{ep['scenario']}/{ep['rep']}")
    healer = subprocess.Popen(
        ["taskset", "-c", cores, str(HEALER), "--ros-args", "-p", f"spec:={d / 'spec'}", "-p", f"policy:={policy}",
         "-p", f"event_log:={d / 'events.jsonl'}", "-p", f"snapshot_log:={d / 'snap.jsonl'}", "-p", f"log_dir:={d / 'logs'}"],
        stdout=open(d / "healer.out", "w"), stderr=subprocess.STDOUT, env=env)
    probe = subprocess.Popen(
        ["taskset", "-c", PROBE_CORES, sys.executable, str(PROBE), "--topics", ",".join(topics), "--inject", "" if real else ep["inject"],
         "--t-inject", str(t_inject), "--t-end", str(t_end), "--out", str(d / "probe.json")],
        stdout=open(d / "probe.out", "w"), stderr=subprocess.STDOUT, env=env)
    inj.start()
    # Healer overhead over a fault-free window: [warm-4, warm].
    time.sleep(max(0.0, t_inject - 4 - time.time()))
    c0, w0 = proc_cpu_s(healer.pid), time.time()
    time.sleep(max(0.0, t_inject - 0.05 - time.time()))
    c1, w1 = proc_cpu_s(healer.pid), time.time()
    rss = proc_rss_mb(healer.pid)
    probe.wait(timeout=post + warm + 30)
    healer.send_signal(signal.SIGINT)
    try:
        healer.wait(timeout=10)
    except subprocess.TimeoutExpired:
        healer.kill()
        healer.wait()
    inj.join(timeout=5)
    inj.close()
    stray = reap(str(d))
    if real:
        pr = json.loads((d / "probe.json").read_text())
        pr.update(t_inject=inj.injected_at, inject_subs=inj.hits, injector=inj.record())
        (d / "probe.json").write_text(json.dumps(pr))
    meta = dict(ep, slot=slot, stray=stray, t_start=t_start, healer_cpu_frac=(c1 - c0) / (w1 - w0), healer_rss_mb=rss,
                healer_rc=healer.returncode, probe_rc=probe.returncode)
    (d / "meta.json").write_text(json.dumps(meta))
    return meta


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", required=True, choices=sorted(SETS))
    ap.add_argument("--policies", default="none,respawn,watchdog,helix")
    ap.add_argument("--scenarios", default="", help="comma separated subset")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--slots", type=int, default=5)
    ap.add_argument("--warm", type=float, default=7.0)
    ap.add_argument("--post", type=float, default=25.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    if not HEALER.exists():
        sys.exit(f"build first: {HEALER} missing")
    out = Path(a.out)
    if out.exists() and any(out.iterdir()):
        sys.exit(f"{out} is not empty; use a fresh directory")
    pick = set(filter(None, a.scenarios.split(",")))
    scen = [s for s in SETS[a.set] if not pick or s.name in pick]
    eps = []
    for rep, s, pol in itertools.product(range(a.reps), scen, a.policies.split(",")):
        eps.append(dict(set=a.set, scenario=s.name, inject=s.inject, root=s.root, expect=list(s.expect),
                        restorable=s.restorable, collateral=list(s.collateral), policy=pol, rep=rep, dir=str(out / f"{s.name}__{pol}__{rep}")))
    print(f"{len(eps)} episodes, {a.slots} slots, ~{len(eps) * (a.warm + a.post + 3) / a.slots / 60:.1f} min", flush=True)
    free = list(range(a.slots))
    done = 0

    def job(ep: dict) -> dict:
        slot = free.pop()
        try:
            return run_episode(ep, slot, a.warm, a.post)
        finally:
            free.append(slot)

    with ThreadPoolExecutor(a.slots) as ex:
        for meta in ex.map(job, eps):
            done += 1
            print(f"[{done}/{len(eps)}] {meta['scenario']} {meta['policy']} rep{meta['rep']} rc={meta['healer_rc']}",
                  flush=True)
    left = subprocess.run(["pgrep", "-c", "-f", "helix_demo/demo_node"], capture_output=True, text=True).stdout.strip()
    print(f"stray demo_node processes after run: {left or 0}; episode processes reaped after healer exit: "
          f"{sum(json.loads((Path(e['dir']) / 'meta.json').read_text())['stray'] for e in eps)}")


if __name__ == "__main__":
    main()
