"""Fault scenarios with ground truth.

Each scenario injects one fault into one node at a fixed time. `root` is the node
the fault was injected into and `expect` the HELIX fault labels that count as a
correct diagnosis. Which repair can fix each synthetic fault is fixed by the
helix_demo implementation (see docs/v3/method.md): a restart fixes every
non-persistent fault, a lifecycle reset fixes degrade and endpoint_loss, nothing
but a backup fixes device_missing, and nothing at all fixes qos_mismatch (a
deployed-config error), so its best achievable outcome is a safe stop.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Scenario:
    name: str
    inject: str  # node:fault[:arg], "" = nominal
    root: str  # "" = nominal
    expect: tuple[str, ...]
    restorable: bool = True
    # Nodes that legitimately need their own repair as a consequence of the root
    # fault (a downstream node that latches and never recovers by itself).
    collateral: tuple[str, ...] = ()


# Development topology: src/helix_healer/config/nav.spec. All tuning happened on this set.
NAV = [
    Scenario("nominal", "", "", ()),
    Scenario("crash_segv", "odom:segv", "odom", ("crash",)),
    Scenario("crash_exit", "localizer:exit:3", "localizer", ("crash",)),
    Scenario("crash_loop", "lidar:device_missing", "lidar", ("crash_loop",)),
    Scenario("hang", "localizer:hang", "localizer", ("hang",)),
    Scenario("hang_lifecycle", "controller:hang", "controller", ("hang",)),
    Scenario("degrade", "odom:degrade:0.3", "odom", ("degraded",)),
    Scenario("degrade_lifecycle", "controller:degrade:0.2", "controller", ("degraded",)),
    Scenario("endpoint_loss", "planner:endpoint_loss", "planner", ("endpoint_loss",)),
    Scenario("endpoint_loss_lifecycle", "lidar:endpoint_loss", "lidar", ("endpoint_loss",)),
    Scenario("cpu_runaway", "odom:cpu_hog:2", "odom", ("cpu_runaway",)),
    Scenario("leak", "localizer:leak:60", "localizer", ("memory_leak",)),
    Scenario("qos_mismatch", "localizer:qos_mismatch", "localizer", ("qos_mismatch",), restorable=False),
]

# Held-out topology: src/helix_healer/config/manip.spec. Written before any DEV
# result was seen; run once, after pre-registration, with no tuning in between.
# Different node names, rates, depth, fault parameters, plus two fault patterns
# DEV never had: a latching downstream node and a crash loop with no backup.
MANIP = [
    Scenario("nominal", "", "", ()),
    Scenario("crash_segv", "joints:segv", "joints", ("crash",)),
    Scenario("crash_exit", "tracker:exit:1", "tracker", ("crash",)),
    Scenario("crash_loop_backup", "camera:device_missing", "camera", ("crash_loop",), collateral=("detector",)),
    Scenario("crash_loop_nobackup", "arm_controller:device_missing", "arm_controller", ("crash_loop",),
             restorable=False),
    Scenario("hang", "tracker:hang", "tracker", ("hang",)),
    Scenario("hang_lifecycle", "arm_planner:hang", "arm_planner", ("hang",)),
    Scenario("hang_latch", "camera:hang", "camera", ("hang",), collateral=("detector",)),
    Scenario("degrade", "joints:degrade:0.4", "joints", ("degraded",)),
    Scenario("degrade_lifecycle", "camera:degrade:0.25", "camera", ("degraded",)),
    Scenario("endpoint_loss", "gripper:endpoint_loss", "gripper", ("endpoint_loss",)),
    Scenario("endpoint_loss_lifecycle", "arm_planner:endpoint_loss", "arm_planner", ("endpoint_loss",)),
    Scenario("cpu_runaway", "arm_controller:cpu_hog:3", "arm_controller", ("cpu_runaway",)),
    Scenario("leak", "detector:leak:40", "detector", ("memory_leak",)),
    Scenario("qos_mismatch", "detector:qos_mismatch", "detector", ("qos_mismatch",), restorable=False),
]

SETS = {"nav": NAV, "manip": MANIP}
