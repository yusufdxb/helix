"""Faults on the real processes of a real ROS 2 stack. No helix_demo code, no fault hooks.

The injector runs as a thread inside the episode runner, outside both the healer and the
probe. It finds its target by the exact `__node:=NAME` (or `X:__node:=NAME`) argv token
of a process in the episode's ROS_DOMAIN_ID, and acts on that PID with signals or with
the stock ros2 CLI. Fault strings:

  kill:NODE:SIG              one signal (SEGV, ABRT, ...) to the live process: a real crash.
  stop:NODE                  SIGSTOP, held while that process lives: every thread frozen.
  duty:NODE:ON:PERIOD        SIGCONT/SIGSTOP duty cycle on the process found at injection
                             (ON = fraction running): emulated CPU starvation. The new
                             process after a restart is not throttled.
  stall:NODE                 the node's stdout/stderr is a FIFO this module owns; the
                             injector stops draining it and fills it, so the node's next
                             per-message log write blocks inside its timer callback. The
                             process is alive and idle, the callback is stalled. The pipe
                             drains again once the stalled process has exited.
  ctl:CM:CONTROLLER:STATE    `ros2 control set_controller_state CONTROLLER STATE -c /CM`.
  loop:NODE:SIG:DELAY        always-on: every process of NODE gets SIG DELAY s after it is
                             first seen, until the episode ends (emulates a fault that a
                             restart cannot clear, such as a missing device).
  flaky:NODE:SIG:LO:HI:QUIET rare: SIG at random U(LO, HI) s intervals, seeded per
                             (scenario, rep) so every arm sees the same schedule; no
                             crash in the last QUIET s.
  param:NODE:NAME:VALUE      `ros2 param set /NODE NAME VALUE` on the live node: runtime
                             parameter corruption, the process keeps running.
  deploy:NODE:FILE:OP[:ARGS] a config deploy: the episode stack's FILE is rewritten
                             atomically, then NODE gets SIGTERM as an operator restart
                             would send. OP is corrupt_urdf (robot_description cut to half,
                             so the URDF no longer parses) or set:KEY:VALUE (one YAML key).
"""

from __future__ import annotations

import errno
import os
import random
import re
import select
import signal
import subprocess
import threading
import time
from pathlib import Path

SIGS = {"SEGV": signal.SIGSEGV, "ABRT": signal.SIGABRT, "KILL": signal.SIGKILL, "TERM": signal.SIGTERM,
        "BUS": signal.SIGBUS, "FPE": signal.SIGFPE}


def _cmdline(pid: int) -> list[str]:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    except OSError:
        return []


def _domain(pid: int) -> str | None:
    try:
        for kv in Path(f"/proc/{pid}/environ").read_bytes().split(b"\0"):
            if kv.startswith(b"ROS_DOMAIN_ID="):
                return kv[14:].decode()
    except OSError:
        pass
    return None


def alive(pid: int) -> bool:
    """True while the process exists and is not a zombie."""
    try:
        st = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except (OSError, IndexError):
        return False
    return st not in ("Z", "X")


def find_pids(node: str, domain: str) -> list[int]:
    want, suffix = f"__node:={node}".encode(), f":__node:={node}".encode()
    out = []
    for p in os.listdir("/proc"):
        if not p.isdigit():
            continue
        pid = int(p)
        if any(t == want or t.endswith(suffix) for t in _cmdline(pid)) and _domain(pid) == domain and alive(pid):
            out.append(pid)
    return sorted(out)


class StallFifo:
    """A FIFO standing in for a node's log file, drained by a thread until stalled."""

    def __init__(self, path: Path, sink: Path) -> None:
        if path.exists():
            path.unlink()
        os.mkfifo(path)
        # O_RDWR: we are a reader from the start, so the node's O_WRONLY open never blocks.
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self.sink = open(sink, "ab")
        self.draining = threading.Event()
        self.draining.set()
        self.done = threading.Event()
        self.t = threading.Thread(target=self._drain, daemon=True)
        self.t.start()

    def _drain(self) -> None:
        while not self.done.is_set():
            if not self.draining.wait(0.05):
                continue
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if r and self.draining.is_set():
                try:
                    self.sink.write(os.read(self.fd, 65536))
                except BlockingIOError:
                    pass

    def stall(self) -> int:
        """Stop draining and fill the pipe. Returns the number of filler bytes written."""
        self.draining.clear()
        time.sleep(0.06)  # let an in-flight read finish
        n = 0
        chunk = b"#" * 4096
        while True:
            try:
                n += os.write(self.fd, chunk)
            except BlockingIOError:
                break
            except OSError as e:
                if e.errno != errno.EAGAIN:
                    raise
                break
        # Top up byte-wise: a pipe accepts partial writes below PIPE_BUF only when room remains.
        while True:
            try:
                n += os.write(self.fd, b"#")
            except BlockingIOError:
                break
        return n

    def resume(self) -> None:
        self.draining.set()

    def close(self) -> None:
        self.done.set()
        self.draining.set()
        self.t.join(timeout=1)
        os.close(self.fd)
        self.sink.close()


class Injector(threading.Thread):
    def __init__(self, fault: str, domain: str, env: dict, t_inject: float, t_end: float, logs: Path, seed: str) -> None:
        super().__init__(daemon=True)
        self.fault, self.domain, self.env = fault, domain, env
        self.t_inject, self.t_end = t_inject, t_end
        self.rng = random.Random(seed)
        self.parts = fault.split(":") if fault else []
        self.kind = self.parts[0] if self.parts else ""
        self.fifo = None
        self.stack = logs.parent / "stack"  # the episode's generated stack (configs live here)
        if self.kind == "stall":
            self.fifo = StallFifo(logs / f"{self.parts[1]}.log", logs / f"{self.parts[1]}.drained.log")
        self.injected_at: float | None = None
        self.hits = 0  # processes the fault actually reached
        self.events: list[tuple[float, str, int]] = []
        self.stopped: set[int] = set()
        self.error = ""

    def _log(self, what: str, pid: int) -> None:
        self.events.append((time.time(), what, pid))

    def _signal(self, pid: int, sig: int, what: str) -> bool:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            return False
        if sig == signal.SIGSTOP:
            self.stopped.add(pid)
        elif sig == signal.SIGCONT:
            self.stopped.discard(pid)
        self._log(what, pid)
        return True

    def _wait_until(self, t: float) -> None:
        while (d := t - time.time()) > 0:
            time.sleep(min(d, 0.05))

    def run(self) -> None:
        try:
            if self.kind:
                self._wait_until(self.t_inject)
                getattr(self, "_f_" + self.kind)(*self.parts[1:])
        except Exception as e:  # noqa: BLE001 - recorded in probe.json, never silent
            self.error = f"{type(e).__name__}: {e}"
        finally:
            for pid in list(self.stopped):
                self._signal(pid, signal.SIGCONT, "cleanup_cont")
            if self.fifo:
                self.fifo.resume()

    def _find(self, node: str) -> int | None:
        pids = find_pids(node, self.domain)
        return pids[0] if len(pids) == 1 else None

    # One-shot crash.
    def _f_kill(self, node: str, sig: str) -> None:
        pid = self._find(node)
        self.injected_at = time.time()
        if pid and self._signal(pid, SIGS[sig], "kill_" + sig):
            self.hits = 1

    def _f_stop(self, node: str) -> None:
        pid = self._find(node)
        self.injected_at = time.time()
        if pid and self._signal(pid, signal.SIGSTOP, "stop"):
            self.hits = 1
            while alive(pid) and time.time() < self.t_end:
                time.sleep(0.02)

    def _f_duty(self, node: str, on: str, period: str) -> None:
        pid = self._find(node)
        self.injected_at = time.time()
        if not pid:
            return
        on_s, off_s = float(on) * float(period), (1 - float(on)) * float(period)
        t = time.time()
        while alive(pid) and t < self.t_end:
            if not self._signal(pid, signal.SIGSTOP, "duty_stop"):
                break
            self.hits = 1
            t += off_s
            self._wait_until(t)
            if not self._signal(pid, signal.SIGCONT, "duty_cont"):
                break
            t += on_s
            self._wait_until(t)

    def _f_stall(self, node: str) -> None:
        pid = self._find(node)
        self.injected_at = time.time()
        if not pid:
            return
        n = self.fifo.stall()
        self.hits = 1
        self._log(f"stall_fill_{n}", pid)
        while alive(pid) and time.time() < self.t_end:
            time.sleep(0.02)
        self.fifo.resume()
        self._log("stall_drain", pid)

    def _f_ctl(self, cm: str, controller: str, state: str) -> None:
        if not self._find(cm):
            self.injected_at = time.time()
            return
        # The fault takes effect while the CLI runs; latency is measured from the CLI start,
        # which overstates it by the CLI's own startup time for every arm alike.
        self.injected_at = time.time()
        r = subprocess.run(["ros2", "control", "set_controller_state", controller, state, "-c", "/" + cm],
                           env=self.env, capture_output=True, text=True, timeout=20)
        self._log(f"ctl_rc{r.returncode}", 0)
        if r.returncode == 0:
            self.hits = 1
        else:
            self.error = (r.stdout + r.stderr).strip()[-300:]

    def _f_loop(self, node: str, sig: str, delay: str) -> None:
        self.injected_at = time.time()
        seen: dict[int, float] = {}
        while time.time() < self.t_end:
            now = time.time()
            for pid in find_pids(node, self.domain):
                seen.setdefault(pid, now if self.hits else now - float(delay))  # first hit is immediate
                if now - seen[pid] >= float(delay) and self._signal(pid, SIGS[sig], "loop_" + sig):
                    self.hits += 1
                    seen[pid] = float("inf")
            time.sleep(0.02)

    def _f_flaky(self, node: str, sig: str, lo: str, hi: str, quiet: str) -> None:
        self.injected_at = time.time()
        t = self.injected_at
        last = self.t_end - float(quiet)
        while t < last:
            self._wait_until(t)
            pid = self._find(node)
            if pid and self._signal(pid, SIGS[sig], "flaky_" + sig):
                self.hits += 1
            t += self.rng.uniform(float(lo), float(hi))

    def _f_param(self, node: str, name: str, value: str) -> None:
        if not self._find(node):
            self.injected_at = time.time()
            return
        r = subprocess.run(["ros2", "param", "set", "--no-daemon", "--spin-time", "3", "/" + node, name, value],
                           env=self.env, capture_output=True, text=True, timeout=20)
        self.injected_at = time.time()  # the set returns once the node applied it (CLI startup takes ~6 s)
        self._log(f"param_rc{r.returncode}", 0)
        if r.returncode == 0 and "Set parameter successful" in r.stdout:
            self.hits = 1
        else:
            self.error = (r.stdout + r.stderr).strip()[-300:]

    def _f_deploy(self, node: str, fname: str, op: str, *args: str) -> None:
        path = self.stack / fname
        text = path.read_text()
        if op == "corrupt_urdf":
            m = re.search(r'^(\s*robot_description: )"(.*)"$', text, re.M)
            if not m:
                raise ValueError(f"{fname}: no robot_description")
            body = m.group(2)[: len(m.group(2)) // 2].rstrip("\\")  # never end inside an escape
            new = text[: m.start()] + m.group(1) + '"' + body + '"' + text[m.end():]
        elif op == "set" and len(args) == 2:
            new, n = re.subn(rf"^(\s*{re.escape(args[0])}:\s*).*$", lambda m: m.group(1) + args[1], text, flags=re.M)
            if n != 1:
                raise ValueError(f"{fname}: key {args[0]} matched {n} times")
        else:
            raise ValueError(f"deploy: unknown op {op}{args}")
        pid = self._find(node)
        tmp = path.with_suffix(path.suffix + ".deploy")
        tmp.write_text(new)
        os.replace(tmp, path)
        self.injected_at = time.time()
        self._log("deploy_" + op, 0)
        if pid and self._signal(pid, signal.SIGTERM, "deploy_TERM"):
            self.hits = 1

    def close(self) -> None:
        if self.fifo:
            self.fifo.close()

    def record(self) -> dict:
        return dict(fault=self.fault, injected_at=self.injected_at, hits=self.hits, error=self.error,
                    events=[(round(t, 4), w, p) for t, w, p in self.events])
