#!/usr/bin/env python3
"""
Orchestrate a closed-loop sim scenario:
  1. launch file brings up HELIX nodes + twist_mux
  2. drive lifecycle transitions to ACTIVE
  3. start ros2 bag recording
  4. spawn inject_lidar_rate_drop.py
  5. wait for scenario duration
  6. terminate, archive artifacts
"""
import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

HELIX_LIFECYCLE_NODES = [
    '/helix_anomaly_detector',
    '/helix_heartbeat_monitor',
    '/helix_log_parser',
    '/helix_context_buffer',
    '/helix_diagnosis_node',
    '/helix_recovery_node',
]


def _transition(node: str, t: str) -> None:
    subprocess.run(['ros2', 'lifecycle', 'set', node, t], check=False)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=90.0)
    p.add_argument('--schedule', default='30:10,20:1,30:10')
    p.add_argument('--artifact-dir', default=f'results/sim_run_{int(time.time())}')
    args = p.parse_args()

    artifact_dir = Path(args.artifact_dir)
    artifact_dir.mkdir(parents=True, exist_ok=True)

    print('[scenario] launching helix_sim_closed_loop')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'launch/helix_sim_closed_loop.launch.py'],
        preexec_fn=os.setsid)
    time.sleep(5.0)

    print('[scenario] configuring + activating lifecycle nodes')
    for n in HELIX_LIFECYCLE_NODES:
        _transition(n, 'configure')
    time.sleep(1.0)
    for n in HELIX_LIFECYCLE_NODES:
        _transition(n, 'activate')
    time.sleep(1.0)

    print(f'[scenario] recording bag to {artifact_dir}/bag')
    bag = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', str(artifact_dir / 'bag'),
         '/helix/faults', '/helix/recovery_hints', '/helix/recovery_actions',
         '/helix/explanations', '/helix/cmd_vel', '/cmd_vel', '/utlidar/cloud_throttled'],
        preexec_fn=os.setsid)

    print(f'[scenario] injecting lidar rate drop: {args.schedule}')
    injector = subprocess.Popen(
        ['python3', 'scripts/sim_faults/inject_lidar_rate_drop.py',
         '--schedule', args.schedule],
        preexec_fn=os.setsid)

    print(f'[scenario] running for {args.duration}s')
    time.sleep(args.duration)

    print('[scenario] terminating')
    for proc in (injector, bag, launch):
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    for proc in (injector, bag, launch):
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    print(f'[scenario] artifacts at {artifact_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
