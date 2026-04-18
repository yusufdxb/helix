#!/usr/bin/env python3
"""
Benchmark helper: launch the C++ helix_topic_rate_monitor executable
subscribed only to /utlidar/imu, transition it to active, sample its
CPU over `duration` seconds with psutil, then shut down cleanly.

Usage: bench_cpp.py --duration 60 --warmup 5
Assumes `ros2`, `source install/setup.bash`, and `ros2 lifecycle` are
available in the caller's shell environment.
"""
import argparse
import subprocess
import time

import psutil


def wait_until_node_ready(node_name: str, timeout_s: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        r = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True, text=True, timeout=5,
        )
        if f"/{node_name}" in r.stdout:
            return True
        time.sleep(0.2)
    return False


def lifecycle_set(node_name: str, transition: str) -> int:
    r = subprocess.run(
        ["ros2", "lifecycle", "set", f"/{node_name}", transition],
        capture_output=True, text=True, timeout=10,
    )
    return r.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--warmup", type=float, default=5.0)
    ap.add_argument("--node-executable", default="helix_topic_rate_monitor")
    ap.add_argument("--package", default="helix_adapter_cpp")
    args = ap.parse_args()

    node_name = "helix_topic_rate_monitor"
    # Launch the C++ node as a subprocess. Restrict topics to /utlidar/imu
    # so we isolate the IMU callback cost (1 active subscription).
    cmd = [
        "ros2", "run", args.package, args.node_executable,
        "--ros-args",
        "-p", "window_sec:=5.0",
        "-p", "publish_period_sec:=0.5",
        "-p", "topics:=[/utlidar/imu]",
        "-p", "sim_mode:=false",
    ]
    proc = subprocess.Popen(cmd)
    try:
        if not wait_until_node_ready(node_name):
            raise RuntimeError(f"{node_name} did not appear in node list")
        # Lifecycle: unconfigured -> inactive
        if lifecycle_set(node_name, "configure") != 0:
            raise RuntimeError("configure failed")
        # inactive -> active
        if lifecycle_set(node_name, "activate") != 0:
            raise RuntimeError("activate failed")

        # `ros2 run` forks the executable as a child process; proc.pid is
        # the Python launcher, not the C++ binary. Walk down to the real
        # process so psutil measures CPU for the C++ executable itself.
        parent = psutil.Process(proc.pid)
        kids = parent.children(recursive=True)
        target_pid = proc.pid
        for k in kids:
            try:
                if "helix_topic_rate_monitor" in " ".join(k.cmdline()):
                    target_pid = k.pid
                    break
            except psutil.NoSuchProcess:
                pass
        print(f"debug: launcher_pid={proc.pid} measured_pid={target_pid}",
              flush=True)
        ps = psutil.Process(target_pid)
        ps.cpu_percent(interval=None)  # prime

        # Warmup (DDS discovery + window fill)
        time.sleep(args.warmup)
        ps.cpu_percent(interval=None)

        t0 = time.monotonic()
        samples = []
        while time.monotonic() - t0 < args.duration:
            samples.append(ps.cpu_percent(interval=1.0))
        wall = time.monotonic() - t0

        active_samples = [s for s in samples if s > 0.0]
        mean_cpu = sum(active_samples) / max(1, len(active_samples))
        peak_cpu = max(samples) if samples else 0.0

        print(
            f"cpp wall_s={wall:.2f} cpu_mean_pct={mean_cpu:.2f} "
            f"cpu_peak_pct={peak_cpu:.2f} samples={len(samples)}",
            flush=True,
        )
    finally:
        # Best-effort graceful shutdown, then kill the whole process tree.
        lifecycle_set(node_name, "deactivate")
        lifecycle_set(node_name, "cleanup")
        try:
            parent = psutil.Process(proc.pid)
            for child in parent.children(recursive=True):
                try:
                    child.terminate()
                except psutil.NoSuchProcess:
                    pass
        except psutil.NoSuchProcess:
            pass
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        # Ensure any orphan matching the executable name is gone.
        subprocess.run(
            ["pkill", "-f", "helix_topic_rate_monitor"],
            capture_output=True, timeout=5,
        )
        time.sleep(1.0)


if __name__ == "__main__":
    main()
