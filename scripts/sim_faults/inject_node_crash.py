#!/usr/bin/env python3
"""Kill a named ROS 2 node after N seconds (for HeartbeatMonitor R4 tests).

Selection is deliberately strict. `pgrep -af <node>` also matches this
script's own command line, the pytest process that spawned it, and any shell
whose arguments mention the node, so a naive first-match kill takes down the
harness instead of the node and the test then reports "no CRASH observed"
while the node is still happily running.

A real ROS 2 node process is the launcher-installed executable, so match on
`<something>/lib/<package>/<node>` and reject anything else.
"""
import argparse
import os
import re
import subprocess
import sys
import time


# `ps -eo args=` truncates each command line to the terminal width, which is
# 80 columns when stdout is not a wide tty (pytest, CI). A launcher-installed
# node path is longer than that, so the executable name gets cut off and the
# node becomes invisible: the search finds nothing while the node is running
# happily. `-ww` disables the truncation and is the whole reason this is a
# named constant rather than an inline list.
PS_COMMAND = ['ps', '-eww', '-o', 'pid=,args=']


def find_node_pids(node: str) -> list[int]:
    """Return pids of processes that are actually the installed node executable."""
    result = subprocess.run(PS_COMMAND, capture_output=True, text=True)
    # The executable lives at .../lib/<package>/<node> and is either exec'd
    # directly or run by an interpreter, so the path may not be argv[0].
    pattern = re.compile(rf'(^|\s|/)lib/[^/\s]+/{re.escape(node)}(\s|$)')
    excluded = {os.getpid(), os.getppid()}
    pids = []
    for line in result.stdout.splitlines():
        pid_text, _, args = line.strip().partition(' ')
        if not pid_text.isdigit():
            continue
        pid = int(pid_text)
        if pid in excluded or 'inject_node_crash' in args:
            continue
        if pattern.search(args):
            pids.append(pid)
    return pids


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', required=True,
                        help='node executable to kill (e.g., helix_context_buffer)')
    parser.add_argument('--after', type=float, default=5.0)
    parser.add_argument('--signal', default='TERM', choices=['TERM', 'KILL'])
    parser.add_argument(
        '--wait', type=float, default=20.0,
        help='Seconds to keep looking for the node after --after elapses. A '
             'launch brings lifecycle nodes up at its own pace, so a fixed '
             'sleep races startup and reports a false "node not found".')
    args = parser.parse_args()

    print(f'[inject_node_crash] sleeping {args.after}s then killing {args.node}', flush=True)
    time.sleep(args.after)

    deadline = time.monotonic() + args.wait
    pids = find_node_pids(args.node)
    while not pids and time.monotonic() < deadline:
        time.sleep(0.5)
        pids = find_node_pids(args.node)

    if not pids:
        print(f'[inject_node_crash] no running node process matches {args.node}',
              file=sys.stderr)
        probe = subprocess.run(PS_COMMAND, capture_output=True, text=True)
        mentions = [ln for ln in probe.stdout.splitlines() if args.node in ln]
        print(f'[inject_node_crash] ps returned {len(probe.stdout.splitlines())} lines, '
              f'{len(mentions)} mention {args.node}', file=sys.stderr)
        for line in mentions[:5]:
            print(f'[inject_node_crash]   {line.strip()[:160]}', file=sys.stderr)
        return 1
    if len(pids) > 1:
        print(f'[inject_node_crash] refusing to guess between pids {pids} for {args.node}',
              file=sys.stderr)
        return 1

    pid = pids[0]
    print(f'[inject_node_crash] SIG{args.signal} to pid {pid}', flush=True)
    subprocess.run(['kill', f'-{args.signal}', str(pid)], check=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
