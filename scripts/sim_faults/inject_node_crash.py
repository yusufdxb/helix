#!/usr/bin/env python3
"""Kill a named ROS 2 node after N seconds (for HeartbeatMonitor R4 tests)."""
import argparse
import subprocess
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--node', required=True, help='node name to kill (e.g., helix_context_buffer)')
    p.add_argument('--after', type=float, default=5.0)
    args = p.parse_args()

    print(f'[inject_node_crash] sleeping {args.after}s then killing {args.node}')
    time.sleep(args.after)
    result = subprocess.run(['pgrep', '-af', args.node], capture_output=True, text=True)
    lines = [ln for ln in result.stdout.splitlines() if args.node in ln]
    if not lines:
        print(f'[inject_node_crash] no process matches {args.node}')
        return 1
    pid = int(lines[0].split()[0])
    print(f'[inject_node_crash] SIGTERM to pid {pid}')
    subprocess.run(['kill', '-TERM', str(pid)])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
