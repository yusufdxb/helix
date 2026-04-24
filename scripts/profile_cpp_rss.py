#!/usr/bin/env python3
"""
Profile RSS breakdown for the C++ anomaly detector node.

Launches the C++ (helix_sensing_cpp) and Python (helix_core) anomaly
detector nodes in isolation, samples /proc/<pid>/status every ~1 s for
a configurable duration, and prints a side-by-side summary with a
theoretical-minimum breakdown.

IMPORTANT: `ros2 run` spawns a shell wrapper around the actual node
process. This script follows the child-process tree to measure the
real node PID, not the wrapper.

Works on x86 (mewtwo) without a live robot -- the nodes start, subscribe
to empty topics, and sit idle.  We measure *startup* RSS, which on
Session 8 hardware was 19.8 MB for C++ and ~45 MB for Python, so idle
measurement is representative of the rclcpp/rclpy runtime baseline.

Usage:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 scripts/profile_cpp_rss.py [--duration 30] [--interval 1.0]

Requires: ROS 2 Humble, helix_sensing_cpp and helix_core built and
sourced (colcon build --symlink-install).
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# -- Constants ----------------------------------------------------------------

# Theoretical memory model parameters (documented in design doc
# CPP_PORT_DESIGN_ANOMALY_DETECTOR.md and Session 8 parity summary).
WINDOW_SIZE = 60           # default param, samples per metric
SIZEOF_DOUBLE = 8          # bytes
DEQUE_BLOCK_BYTES = 512    # libstdc++ deque<double> block size
METRIC_STATE_OVERHEAD = 64 # MetricState struct fields + padding
NUM_METRICS_HW = 6         # typical GO2 idle metric count

# Services created by rclcpp_lifecycle::LifecycleNode.
LIFECYCLE_SERVICES = 6     # change_state, get_state, etc.
PARAMETER_SERVICES = 5     # describe_parameters, get_parameters, etc.


@dataclass
class RssSample:
    """One point-in-time RSS reading from /proc/<pid>/status."""
    elapsed_s: float
    vm_rss_kb: int
    vm_size_kb: int


@dataclass
class SmapsBreakdown:
    """Memory breakdown from /proc/<pid>/smaps_rollup."""
    rss_kb: int = 0
    pss_kb: int = 0
    shared_clean_kb: int = 0
    shared_dirty_kb: int = 0
    private_clean_kb: int = 0
    private_dirty_kb: int = 0
    num_shared_libs: int = 0


@dataclass
class NodeProfile:
    """Collected RSS samples for one node process."""
    label: str
    pid: int
    wrapper_pid: int = 0
    cmdline: str = ""
    samples: List[RssSample] = field(default_factory=list)
    smaps: Optional[SmapsBreakdown] = None

    @property
    def rss_mb_values(self) -> List[float]:
        return [s.vm_rss_kb / 1024.0 for s in self.samples]

    @property
    def rss_mean_mb(self) -> float:
        vals = self.rss_mb_values
        return sum(vals) / len(vals) if vals else 0.0

    @property
    def rss_min_mb(self) -> float:
        vals = self.rss_mb_values
        return min(vals) if vals else 0.0

    @property
    def rss_max_mb(self) -> float:
        vals = self.rss_mb_values
        return max(vals) if vals else 0.0

    @property
    def rss_first_mb(self) -> float:
        return self.samples[0].vm_rss_kb / 1024.0 if self.samples else 0.0

    @property
    def rss_last_mb(self) -> float:
        return self.samples[-1].vm_rss_kb / 1024.0 if self.samples else 0.0

    @property
    def vmsize_mean_mb(self) -> float:
        vals = [s.vm_size_kb / 1024.0 for s in self.samples]
        return sum(vals) / len(vals) if vals else 0.0


# -- /proc helpers ------------------------------------------------------------

def find_leaf_pid(parent_pid: int) -> int:
    """Walk the process tree to find the leaf child (the actual node process).

    `ros2 run` creates a bash wrapper that exec's the real executable.
    This function follows the child chain to the leaf.
    """
    try:
        children_text = Path(
            f"/proc/{parent_pid}/task/{parent_pid}/children"
        ).read_text().strip()
        if not children_text:
            return parent_pid
        first_child = int(children_text.split()[0])
        return find_leaf_pid(first_child)
    except (FileNotFoundError, ProcessLookupError, PermissionError,
            ValueError, IndexError):
        return parent_pid


def read_proc_status(pid: int) -> Optional[Tuple[int, int]]:
    """Read VmRSS and VmSize from /proc/<pid>/status.

    Returns (rss_kb, vmsize_kb) or None if the process is gone.
    """
    try:
        text = Path(f"/proc/{pid}/status").read_text()
        rss_match = re.search(r"^VmRSS:\s+(\d+)\s+kB", text, re.MULTILINE)
        size_match = re.search(r"^VmSize:\s+(\d+)\s+kB", text, re.MULTILINE)
        if rss_match and size_match:
            return int(rss_match.group(1)), int(size_match.group(1))
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        pass
    return None


def read_cmdline(pid: int) -> str:
    """Read /proc/<pid>/cmdline."""
    try:
        return Path(f"/proc/{pid}/cmdline").read_text().replace("\0", " ").strip()
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return "(unknown)"


def read_smaps_rollup(pid: int) -> Optional[SmapsBreakdown]:
    """Read /proc/<pid>/smaps_rollup for shared/private/PSS breakdown."""
    try:
        text = Path(f"/proc/{pid}/smaps_rollup").read_text()
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None

    breakdown = SmapsBreakdown()
    field_map = {
        "Rss": "rss_kb",
        "Pss": "pss_kb",
        "Shared_Clean": "shared_clean_kb",
        "Shared_Dirty": "shared_dirty_kb",
        "Private_Clean": "private_clean_kb",
        "Private_Dirty": "private_dirty_kb",
    }
    for proc_key, attr in field_map.items():
        m = re.search(rf"^{proc_key}:\s+(\d+)\s+kB", text, re.MULTILINE)
        if m:
            setattr(breakdown, attr, int(m.group(1)))

    # Count shared libraries from /proc/<pid>/maps
    try:
        maps_text = Path(f"/proc/{pid}/maps").read_text()
        libs = set()
        for line in maps_text.splitlines():
            if ".so" in line:
                parts = line.split()
                if len(parts) >= 6:
                    libs.add(parts[-1])
        breakdown.num_shared_libs = len(libs)
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        pass

    return breakdown


# -- Process management -------------------------------------------------------

def launch_node(package: str, executable: str, node_name: str,
                extra_args: Optional[List[str]] = None) -> subprocess.Popen:
    """Launch a ROS 2 node via `ros2 run` and return the Popen handle."""
    cmd = ["ros2", "run", package, executable, "--ros-args",
           "-r", f"__node:={node_name}"]
    if extra_args:
        cmd.extend(extra_args)
    return subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )


def kill_process_group(proc: subprocess.Popen) -> None:
    """Kill the entire process group."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)


# -- Sampling -----------------------------------------------------------------

def sample_loop(pid: int, label: str, duration: float,
                interval: float) -> NodeProfile:
    """Sample /proc/<pid>/status at `interval` for `duration` seconds."""
    profile = NodeProfile(label=label, pid=pid)
    start = time.monotonic()
    deadline = start + duration

    while time.monotonic() < deadline:
        elapsed = time.monotonic() - start
        reading = read_proc_status(pid)
        if reading is None:
            break
        rss_kb, vmsize_kb = reading
        profile.samples.append(RssSample(
            elapsed_s=round(elapsed, 2),
            vm_rss_kb=rss_kb,
            vm_size_kb=vmsize_kb,
        ))
        time.sleep(interval)

    return profile


# -- Reporting ----------------------------------------------------------------

def print_theoretical_breakdown() -> None:
    """Print the theoretical-minimum RSS estimation for the C++ node."""
    per_metric_bytes = METRIC_STATE_OVERHEAD + DEQUE_BLOCK_BYTES
    total_window_kb = NUM_METRICS_HW * per_metric_bytes / 1024

    map_overhead_bytes = 56 + 13 * 8 + NUM_METRICS_HW * (48 + per_metric_bytes)
    map_overhead_kb = map_overhead_bytes / 1024

    ros_io_kb = 21  # 2 subs + 1 pub + DDS queue buffers
    service_overhead_kb = (LIFECYCLE_SERVICES + PARAMETER_SERVICES) * 2

    app_total_kb = total_window_kb + map_overhead_kb + ros_io_kb + service_overhead_kb

    print()
    print("=" * 64)
    print("  THEORETICAL MINIMUM RSS BREAKDOWN (C++ anomaly detector)")
    print("=" * 64)
    print()
    print("  Component                             Estimated")
    print("  " + "-" * 54)
    print("  rclcpp + DDS runtime baseline:        ~12-14 MB")
    print("    - rcl context, allocator, rmw layer")
    print("    - Fast-DDS participant + builtin endpoints")
    print("    - Shared library .text/.rodata pages")
    print("    - rclcpp_lifecycle state machine + services")
    print("    - /rosout publisher (always created)")
    print("  Message type support libs:            ~0.6 MB")
    print("    - diagnostic_msgs, std_msgs, helix_msgs,")
    print("      lifecycle_msgs (4 pkgs x ~150 KB)")
    print(f"  Lifecycle services ({LIFECYCLE_SERVICES}):"
          f"               ~{LIFECYCLE_SERVICES * 2} KB")
    print(f"  Parameter services ({PARAMETER_SERVICES}):"
          f"               ~{PARAMETER_SERVICES * 2} KB")
    print(f"  Sliding window buffers:               ~{total_window_kb:.1f} KB")
    print(f"    - {NUM_METRICS_HW} metrics x {WINDOW_SIZE} samples x"
          f" {SIZEOF_DOUBLE}B = {NUM_METRICS_HW * WINDOW_SIZE * SIZEOF_DOUBLE} B raw")
    print(f"    - std::deque block overhead: {DEQUE_BLOCK_BYTES}B/metric")
    print(f"  unordered_map + MetricState:          ~{map_overhead_kb:.1f} KB")
    print(f"  ROS I/O (2 subs + 1 pub + DDS):      ~{ros_io_kb} KB")
    print("  " + "-" * 54)
    print(f"  App-level data total:                 ~{app_total_kb:.0f} KB"
          f" ({app_total_kb/1024:.2f} MB)")
    print(f"  ESTIMATED TOTAL:                      ~12.7-14.7 MB")
    print()
    print("  Jetson measured (Session 8):             19.8 MB")
    print(f"  Gap vs upper estimate:                  ~5.1 MB")
    print()
    print("  Likely gap sources:")
    print("    - DDS discovery DB + participant heartbeat buffers")
    print("    - malloc arena fragmentation (glibc M_ARENA_MAX=8)")
    print("    - Jetson aarch64 page-size / allocator differences")
    print("    - DDS builtin topic readers (4 default endpoints)")
    print()
    print("  CONCLUSION: The node's own data (windows + map) is < 30 KB.")
    print("  >95% of RSS is rclcpp/DDS runtime overhead, not application")
    print("  logic. The design doc's 8-10 MB estimate was optimistic about")
    print("  the rclcpp_lifecycle + Fast-DDS baseline. Window buffers and")
    print("  node allocations are NOT the bottleneck.")
    print()
    print("  OPTIMIZATION OPPORTUNITIES:")
    print("    1. MALLOC_ARENA_MAX=2: reduces glibc arena fragmentation")
    print("       (default 8 arenas x ~2 MB = ~16 MB overhead; 2 arenas")
    print("       could save ~4-6 MB on Jetson)")
    print("    2. CycloneDDS instead of Fast-DDS: typically ~2-4 MB less")
    print("       baseline RSS due to lighter discovery implementation")
    print("    3. Component container (intra-process): amortize the DDS")
    print("       baseline across multiple nodes, reducing per-node RSS")
    print("    4. None of these require C++ code changes in the node")


def print_smaps_section(profile: NodeProfile) -> None:
    """Print the smaps breakdown for a profile."""
    s = profile.smaps
    if not s:
        return
    rss_mb = s.rss_kb / 1024
    pss_mb = s.pss_kb / 1024
    sc_mb = s.shared_clean_kb / 1024
    sd_mb = s.shared_dirty_kb / 1024
    pc_mb = s.private_clean_kb / 1024
    pd_mb = s.private_dirty_kb / 1024

    print(f"\n  [{profile.label}] Memory map breakdown (smaps_rollup):")
    print(f"    RSS (total):         {rss_mb:7.1f} MB")
    print(f"    PSS (proportional):  {pss_mb:7.1f} MB")
    print(f"    Shared_Clean:        {sc_mb:7.1f} MB  (shared .so .text/.rodata)")
    print(f"    Shared_Dirty:        {sd_mb:7.1f} MB  (shared writable pages)")
    print(f"    Private_Clean:       {pc_mb:7.1f} MB  (file-backed private pages)")
    print(f"    Private_Dirty:       {pd_mb:7.1f} MB  (heap + stack + BSS)")
    print(f"    Shared libraries:    {s.num_shared_libs:4d}")
    print(f"    Unique memory (Private_Dirty) is the true per-process cost.")


def print_report(profiles: List[NodeProfile]) -> None:
    """Print the full measurement report."""
    print()
    print("=" * 64)
    print("  RSS PROFILING RESULTS")
    print("=" * 64)

    for p in profiles:
        if not p.samples:
            print(f"\n  [{p.label}] pid={p.pid}  NO SAMPLES (process exited early)")
            continue

        print(f"\n  [{p.label}]")
        print(f"    Node PID:    {p.pid}  (wrapper PID: {p.wrapper_pid})")
        print(f"    Cmdline:     {p.cmdline[:80]}")
        print(f"    Samples:     {len(p.samples)}")
        print(f"    VmRSS  mean:  {p.rss_mean_mb:7.2f} MB")
        print(f"    VmRSS  min:   {p.rss_min_mb:7.2f} MB")
        print(f"    VmRSS  max:   {p.rss_max_mb:7.2f} MB")
        print(f"    VmRSS  first: {p.rss_first_mb:7.2f} MB")
        print(f"    VmRSS  last:  {p.rss_last_mb:7.2f} MB")
        print(f"    VmSize mean:  {p.vmsize_mean_mb:7.2f} MB")
        drift = p.rss_last_mb - p.rss_first_mb
        print(f"    Drift (last-first): {drift:+.2f} MB")

    # smaps breakdown
    for p in profiles:
        print_smaps_section(p)

    # Side-by-side comparison
    if len(profiles) >= 2:
        cpp = profiles[0]
        py = profiles[1]
        if cpp.samples and py.samples:
            ratio = cpp.rss_mean_mb / py.rss_mean_mb * 100 if py.rss_mean_mb > 0 else 0
            print(f"\n  COMPARISON (C++ vs Python):")
            print(f"    C++ RSS mean:            {cpp.rss_mean_mb:.1f} MB")
            print(f"    Python RSS mean:         {py.rss_mean_mb:.1f} MB")
            print(f"    C++ / Python RSS ratio:  {ratio:.1f}%")
            print(f"    Absolute savings:        {py.rss_mean_mb - cpp.rss_mean_mb:.1f} MB per node")

            if cpp.smaps and py.smaps:
                cpp_pd = cpp.smaps.private_dirty_kb / 1024
                py_pd = py.smaps.private_dirty_kb / 1024
                pd_ratio = cpp_pd / py_pd * 100 if py_pd > 0 else 0
                print(f"    C++ Private_Dirty:       {cpp_pd:.1f} MB")
                print(f"    Python Private_Dirty:    {py_pd:.1f} MB")
                print(f"    Private_Dirty ratio:     {pd_ratio:.1f}%"
                      f" (true per-process savings)")

            if ratio > 30:
                print(f"    Design doc target (30%): MISS (actual {ratio:.0f}%)")
            else:
                print(f"    Design doc target (30%): PASS ({ratio:.0f}%)")

    # Per-sample timeline (compact)
    for p in profiles:
        if not p.samples:
            continue
        print(f"\n  [{p.label}] Timeline (VmRSS):")
        step = max(1, len(p.samples) // 15)  # show at most ~15 rows
        for i, s in enumerate(p.samples):
            if i % step != 0 and i != len(p.samples) - 1:
                continue
            bar_len = int(s.vm_rss_kb / 1024 / 2)
            bar = "#" * bar_len
            print(f"    {s.elapsed_s:6.1f}s  {s.vm_rss_kb/1024:6.1f} MB  {bar}")


# -- Main ---------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Profile RSS of the C++ and Python anomaly detector nodes.")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Sampling duration in seconds (default: 30)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Sampling interval in seconds (default: 1.0)")
    parser.add_argument("--cpp-only", action="store_true",
                        help="Only profile the C++ node")
    parser.add_argument("--python-only", action="store_true",
                        help="Only profile the Python node")
    parser.add_argument("--settle", type=float, default=5.0,
                        help="Seconds to wait after launch before sampling "
                             "(default: 5, enough for Python interpreter init)")
    args = parser.parse_args()

    # Verify ROS 2 environment
    ros_distro = os.environ.get("ROS_DISTRO")
    if not ros_distro:
        print("ERROR: ROS 2 not sourced. Run:\n"
              "  source /opt/ros/humble/setup.bash\n"
              "  source install/setup.bash", file=sys.stderr)
        sys.exit(1)

    print(f"ROS 2 distro: {ros_distro}")
    print(f"RMW: {os.environ.get('RMW_IMPLEMENTATION', '(default = rmw_fastrtps_cpp)')}")
    print(f"Duration: {args.duration}s, interval: {args.interval}s, "
          f"settle: {args.settle}s")

    processes: List[subprocess.Popen] = []
    profiles: List[NodeProfile] = []

    try:
        # -- Launch nodes ------------------------------------------------
        cpp_proc = None
        py_proc = None

        if not args.python_only:
            print("\nLaunching C++ anomaly detector (helix_sensing_cpp)...")
            cpp_proc = launch_node(
                "helix_sensing_cpp", "helix_anomaly_detector",
                "helix_anomaly_detector_cpp_profile")
            processes.append(cpp_proc)
            print(f"  Wrapper PID: {cpp_proc.pid}")

        if not args.cpp_only:
            print("Launching Python anomaly detector (helix_core)...")
            py_proc = launch_node(
                "helix_core", "helix_anomaly_detector",
                "helix_anomaly_detector_py_profile")
            processes.append(py_proc)
            print(f"  Wrapper PID: {py_proc.pid}")

        # -- Settle (let Python interpreter + DDS init finish) -----------
        print(f"\nWaiting {args.settle}s for processes to initialize...")
        time.sleep(args.settle)

        # -- Resolve actual node PIDs ------------------------------------
        # `ros2 run` spawns a shell wrapper; the real node is a child.
        node_pids: Dict[str, Tuple[int, int, str]] = {}  # label -> (node_pid, wrapper_pid, label)

        if cpp_proc and cpp_proc.poll() is None:
            cpp_node_pid = find_leaf_pid(cpp_proc.pid)
            cpp_cmdline = read_cmdline(cpp_node_pid)
            print(f"  C++ node PID: {cpp_node_pid} ({cpp_cmdline[:60]})")
            node_pids["cpp"] = (cpp_node_pid, cpp_proc.pid,
                                "C++ (helix_sensing_cpp)")
        elif cpp_proc:
            print(f"  WARNING: C++ process exited with code {cpp_proc.returncode}")

        if py_proc and py_proc.poll() is None:
            py_node_pid = find_leaf_pid(py_proc.pid)
            py_cmdline = read_cmdline(py_node_pid)
            print(f"  Python node PID: {py_node_pid} ({py_cmdline[:60]})")
            node_pids["py"] = (py_node_pid, py_proc.pid,
                               "Python (helix_core)")
        elif py_proc:
            print(f"  WARNING: Python process exited with code {py_proc.returncode}")

        # -- Sample RSS --------------------------------------------------
        print(f"\nSampling RSS for {args.duration}s...")

        for key in ["cpp", "py"]:
            if key not in node_pids:
                continue
            node_pid, wrapper_pid, label = node_pids[key]
            profile = sample_loop(node_pid, label, args.duration, args.interval)
            profile.wrapper_pid = wrapper_pid
            profile.cmdline = read_cmdline(node_pid)
            profile.smaps = read_smaps_rollup(node_pid)
            profiles.append(profile)

        # -- Report ------------------------------------------------------
        print_report(profiles)
        print_theoretical_breakdown()

    finally:
        print("\nCleaning up...")
        for p in processes:
            kill_process_group(p)
        print("Done.")


if __name__ == "__main__":
    main()
