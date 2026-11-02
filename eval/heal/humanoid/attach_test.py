#!/usr/bin/env python3
"""End-to-end test of attach mode: the healer adopts nodes that `ros2 launch` started.

The humanoid stack (make_stack.py) is started by a real launch file, either with
respawn=False (the healer is the only repairer) or respawn=True (launch and the healer
both repair, the case a community user would hit first). The healer starts afterwards
with a spec whose nodes use attach= patterns instead of exec lines. Two real crashes are
injected on the real PIDs: SIGSEGV to robot_state_publisher, then SIGSEGV to
controller_manager (whose controllers must be re-spawned). Reported:

  attach      nodes the healer adopted
  blackout_s  longest gap on /tf and /joint_states after each crash, from an independent probe
  copies      live processes per node at the end (2 = duplicate started by both repairers)
  orphans     processes of the stack still alive after launch shut down
  survivors   processes still alive after the healer was stopped too

Usage: attach_test.py --mode {norespawn,respawn} --out DIR [--domain 90]
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
from inject import Injector, find_pids  # noqa: E402
from provenance import stamp  # noqa: E402

ROOT = HERE.parents[2]
HEALER = ROOT / "install/helix_healer/lib/helix_healer/helix_healer_node"
PROBE = HERE.parent / "probe.py"
NODES = ("controller_manager", "rsp", "camera", "camera_backup")


def launch_file(stack: Path, respawn: bool) -> str:
    cm = ["/opt/ros/humble/lib/controller_manager/ros2_control_node", "--ros-args", "--params-file",
          f"{stack}/controllers.yaml", "-r", "controller_manager:__node:=controller_manager"]
    rsp = ["/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher", "--ros-args", "--params-file",
           f"{stack}/rsp.yaml", "-r", "__node:=rsp"]
    cam = ["/opt/ros/humble/lib/image_tools/cam2image", "--ros-args", "-p", "burger_mode:=true", "-p",
           "frequency:=30.0", "-r", "__node:=camera", "-p", f"tag:={stack}"]
    spawner = ["/opt/ros/humble/lib/controller_manager/spawner", "joint_state_broadcaster",
               "forward_position_controller", "-c", "/controller_manager"]
    return f"""from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
def generate_launch_description():
    r = {respawn!r}
    return LaunchDescription([
        ExecuteProcess(cmd={cm!r}, respawn=r, output='log'),
        ExecuteProcess(cmd={rsp!r}, respawn=r, output='log'),
        ExecuteProcess(cmd={cam!r}, respawn=r, output='log'),
        TimerAction(period=2.0, actions=[ExecuteProcess(cmd={spawner!r}, output='log')]),
    ])
"""


def attach_spec(stack: Path) -> str:
    return f"""healer startup_grace_s=3 verify_s=5

node controller_manager attach={stack}/controllers.yaml
out controller_manager /joint_states 250
then controller_manager @controller_manager/spawner joint_state_broadcaster forward_position_controller -c /controller_manager

node rsp attach={stack}/rsp.yaml
in rsp /joint_states
out rsp /tf 83

node camera fallback=camera_backup attach=tag:={stack}
out camera /image 30

node camera_backup autostart=0
out camera_backup /image 15
exec camera_backup @image_tools/cam2image --ros-args -p burger_mode:=true -p frequency:=15.0 -r __node:=camera_backup
"""


def blackout(arr: list[float], t0: float, t1: float) -> float:
    """Longest gap consumers saw between t0 and t1, counting the last message before t0."""
    before = [x for x in arr if x < t0]
    pts = ([before[-1]] if before else [t0]) + [x for x in arr if t0 <= x < t1] + [t1]
    return round(max(b - a for a, b in zip(pts, pts[1:])), 3)


def counts(domain: str) -> dict:
    return {n: len(find_pids(n, domain)) for n in NODES}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("norespawn", "respawn"), required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--domain", default="90")
    ap.add_argument("--allow-dirty", action="store_true", help="run an untraceable build (recorded)")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True)
    stamp(out, a.allow_dirty)
    stack = out / "stack"
    subprocess.run([sys.executable, str(HERE / "make_stack.py"), str(stack)], check=True, capture_output=True)
    (out / "launch.py").write_text(launch_file(stack, a.mode == "respawn"))
    spec = attach_spec(stack)
    (out / "spec").write_text(spec)
    env = dict(os.environ, ROS_DOMAIN_ID=a.domain, ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=str(out / "roslog"))
    (out / "logs").mkdir()
    procs = []
    launch = subprocess.Popen(["ros2", "launch", str(out / "launch.py")], env=env, start_new_session=True,
                              stdout=open(out / "launch.log", "w"), stderr=subprocess.STDOUT)
    procs.append(launch)
    time.sleep(8)
    t_h = time.time()
    healer = subprocess.Popen([str(HEALER), "--ros-args", "-p", f"spec:={out / 'spec'}", "-p", "policy:=helix",
                               "-p", f"event_log:={out / 'events.jsonl'}", "-p", f"log_dir:={out / 'logs'}"],
                              env=env, start_new_session=True, stdout=open(out / "healer.log", "w"),
                              stderr=subprocess.STDOUT)
    procs.append(healer)
    t1, t2, t_end = t_h + 8, t_h + 18, t_h + 32
    probe = subprocess.Popen([sys.executable, str(PROBE), "--topics",
                              "/tf=tf2_msgs/msg/TFMessage,/joint_states=sensor_msgs/msg/JointState,"
                              "/image=sensor_msgs/msg/Image", "--inject", "", "--t-inject", str(t1),
                              "--t-end", str(t_end), "--out", str(out / "probe.json")], env=env)
    res = {"mode": a.mode, "before": None}
    time.sleep(max(0, t1 - 0.5 - time.time()))
    res["before"] = counts(a.domain)
    injs = [Injector("kill:rsp:SEGV", a.domain, env, t1, t_end, out / "logs", "a"),
            Injector("kill:controller_manager:SEGV", a.domain, env, t2, t_end, out / "logs", "b")]
    for i in injs:
        i.start()
    for i in injs:
        i.join()
    time.sleep(max(0, t_end - 1 - time.time()))
    res["copies"] = counts(a.domain)
    probe.wait(timeout=30)
    os.killpg(launch.pid, signal.SIGINT)
    try:
        launch.wait(timeout=15)
    except subprocess.TimeoutExpired:
        res["launch_shutdown"] = "timeout"
    time.sleep(2)
    res["orphans"] = counts(a.domain)
    healer.send_signal(signal.SIGINT)
    try:
        healer.wait(timeout=15)
    except subprocess.TimeoutExpired:
        healer.kill()
    time.sleep(1)
    res["survivors"] = counts(a.domain)
    # Clean up anything left so the next test starts empty.
    for n in NODES + ("spawner",):
        for pid in find_pids(n, a.domain):
            os.kill(pid, signal.SIGKILL)
    p = json.loads((out / "probe.json").read_text())
    ar = p["arrivals"]
    res["injected"] = [i.record() for i in injs]
    res["blackout_rsp_crash"] = {t: blackout(ar[t], injs[0].injected_at or t1, t2) for t in ar}
    res["blackout_cm_crash"] = {t: blackout(ar[t], injs[1].injected_at or t2, t_end - 1) for t in ar}
    # Event times are seconds since healer start; the faults land at about 8 and 18.
    ev = [json.loads(x) for x in (out / "events.jsonl").read_text().splitlines()] if (out / "events.jsonl").exists() else []
    res["events"] = [(round(e.get("t", 0), 3), e.get("node"), e.get("kind"), str(e.get("detail", ""))[:70])
                     for e in ev if e.get("kind") not in ("tick",)]
    (out / "result.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if k != "events"}, indent=1))
    for e in res["events"]:
        print(e)


if __name__ == "__main__":
    main()
