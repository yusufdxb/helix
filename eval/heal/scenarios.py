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
    # Agent sets only: the action name (policy.cpp to_string) of the scenario's correct
    # repair class. The oracle arm ranks it first; "correct repair" means it verified.
    repair: str = ""


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

# Real ROS 2 humanoid stack (eval/heal/humanoid/make_stack.py): stock ros2_control with mock
# hardware, robot_state_publisher, image_tools cam2image. Faults act on the real processes
# (eval/heal/humanoid/inject.py); nothing in these nodes was written for HELIX. Labels were
# fixed before any humanoid episode ran. A duty-cycled process stalls for 0.14 s at a time,
# longer than the learned stale limit at 30 Hz and above, so hang is an accepted label there.
HUMANOID = [
    Scenario("nominal", "", "", ()),
    Scenario("crash_cm", "kill:controller_manager:SEGV", "controller_manager", ("crash",)),
    Scenario("crash_rsp", "kill:rsp:ABRT", "rsp", ("crash",)),
    Scenario("crash_camera", "kill:camera:SEGV", "camera", ("crash",)),
    Scenario("hang_rsp", "stop:rsp", "rsp", ("hang",)),
    Scenario("hang_cm", "stop:controller_manager", "controller_manager", ("hang",)),
    Scenario("stall_camera", "stall:camera", "camera", ("hang",)),
    Scenario("degrade_camera", "duty:camera:0.3:0.2", "camera", ("degraded", "hang")),
    Scenario("degrade_cm", "duty:controller_manager:0.3:0.2", "controller_manager", ("degraded", "hang")),
    Scenario("deactivate_jsb", "ctl:controller_manager:joint_state_broadcaster:inactive", "controller_manager",
             ("hang", "endpoint_loss")),
    Scenario("crashloop_camera", "loop:camera:SEGV:0.3", "camera", ("crash_loop",)),
    Scenario("flaky_camera", "flaky:camera:SEGV:4:10:10", "camera", ("crash", "crash_loop")),
    # Near miss: 80% duty keeps every gap under the learned limit and the rate above the
    # degraded ratio. The right answer is no action; any decision counts as unnecessary.
    Scenario("nearmiss_camera", "duty:camera:0.8:0.2", "", ()),
]

# Held-out humanoid faults, same stack, written 2026-11-02 after the HUMANOID set above had
# been used to develop core-dump detection, fresh-message verification and the backoff rule,
# and before any of these ran once. Labels fixed here; see
# docs/preregistration/2026-11-02-humanoid-real.md. rsp has no backup and is critical, so a
# crash loop on it cannot be restored and the right outcome is SAFE_STOP.
HUMANOID_HELDOUT = [
    Scenario("nominal", "", "", ()),
    Scenario("term_rsp", "kill:rsp:TERM", "rsp", ("crash",)),
    Scenario("sigkill_cm", "kill:controller_manager:KILL", "controller_manager", ("crash",)),
    Scenario("hang_camera", "stop:camera", "camera", ("hang",)),
    Scenario("degrade_rsp", "duty:rsp:0.3:0.2", "rsp", ("degraded", "hang")),
    Scenario("flaky_rsp", "flaky:rsp:ABRT:4:10:10", "rsp", ("crash", "crash_loop")),
    Scenario("crashloop_rsp", "loop:rsp:ABRT:0.3", "rsp", ("crash_loop",), restorable=False),
    # Near misses: the right answer is no action, any decision counts as unnecessary.
    # forward_position_controller publishes nothing the healer judges.
    Scenario("deactivate_fwd", "ctl:controller_manager:forward_position_controller:inactive", "", ()),
    Scenario("nearmiss_rsp", "duty:rsp:0.8:0.2", "", ()),
]

# Withheld agent families, same humanoid stack, written 2026-11-02 before any of them ran
# against a healer; see docs/preregistration/agent-withheld.md. Only the injector premises
# were checked, with no healer running. W1 corrupts a parameter on the live node, W2 deploys
# a bad config and restarts the node, safe_deploy_rsp deploys a valid config (keeping it is
# right, reverting it is unsafe). Expect sets use v3's unchanged diagnosis vocabulary; v3 has
# no config label, so a deploy reads as the crash its restart causes, then whatever follows.
# rsp is downstream of controller_manager, so it is collateral when the controller dies.
AGENT_WITHHELD = [
    Scenario("nominal", "", "", ()),
    Scenario("w1_param_degrade", "param:rsp:publish_frequency:5.0", "rsp", ("degraded", "hang"),
             repair="param_restore"),
    Scenario("w1_param_stale", "param:rsp:publish_frequency:0.5", "rsp", ("hang", "degraded"),
             repair="param_restore"),
    Scenario("w2_rsp_bad_urdf", "deploy:rsp:rsp.yaml:corrupt_urdf", "rsp", ("crash", "crash_loop"),
             repair="restart_lkg"),
    Scenario("w2_cm_rate", "deploy:controller_manager:controllers.yaml:set:update_rate:20", "controller_manager",
             ("crash", "degraded"), collateral=("rsp",), repair="restart_lkg"),
    Scenario("w2_cm_bad_urdf", "deploy:controller_manager:controllers.yaml:corrupt_urdf", "controller_manager",
             ("crash", "crash_loop"), collateral=("rsp",), repair="restart_lkg"),
    Scenario("safe_deploy_rsp", "deploy:rsp:rsp.yaml:set:publish_frequency:120.0", "rsp", ("crash",),
             repair="restart"),
]

# Plumbing check for the agent's new repairs, run before the freeze in place of the withheld
# families. The node is humanoid/dev_rate.py (synthetic, added to the humanoid spec only for
# this set). Its episodes verify that ParamRestore, RestartLKG and rollback execute; they are
# never training, history or prompt data.
AGENT_PLUMBING = [
    Scenario("p_param", "param:dev_rate:rate:2.0", "dev_rate", ("degraded", "hang"), repair="param_restore"),
    Scenario("p_deploy_bad", "deploy:dev_rate:dev_rate.yaml:set:rate:abc", "dev_rate", ("crash", "crash_loop"),
             repair="restart_lkg"),
    Scenario("p_deploy_slow", "deploy:dev_rate:dev_rate.yaml:set:rate:5.0", "dev_rate", ("crash", "degraded"),
             repair="restart_lkg"),
    Scenario("p_safe_deploy", "deploy:dev_rate:dev_rate.yaml:set:rate:60.0", "dev_rate", ("crash",), repair="restart"),
]

# Transactional-baseline challenge set (docs/preregistration/txn-baseline.md), same humanoid
# stack. Written before any of it ran against a healer. Multi-root scenarios name every root
# and its repair joined by "+", in the same order. TXN_D are the follow-up delays (s after the
# fresh process is first seen); they are fixed from the nominal LKG commit-latency calibration
# before the freeze, so that early < L(5) < mid < L(10) < late, where L(H) is the time from a
# deploy restart to the LKG commit at agent_lkg_hold_s = H.
#   C1 multi-cause: two independent faults at once, or a bad config that only surfaces when an
#      unrelated crash restarts the node.
#   C2 safe_deploy_crash: a valid deploy, then an unrelated crash of the freshly started
#      process D s later. The repair is a restart; reverting the deploy is unsafe.
#   C3 late_cfg: a deploy whose value crashes the node D s after it starts. The repair is
#      restart_lkg; once the bad config has been committed as LKG it is poisoned, and no arm
#      in this set can restore (late: restorable=False).
TXN_D = {"early": 4.0, "mid": 11.0, "late": 22.0}
_RSP_CRASH = ("crash", "crash_loop")
TXN = [
    Scenario("nominal", "", "", ()),
    Scenario("c1_latent_cfg_crash", "deploylatent:rsp:rsp.yaml:corrupt_urdf+kill:rsp:SEGV@2", "rsp", _RSP_CRASH,
             repair="restart_lkg"),
    Scenario("c1_cm_cfg_camera_hang", "deploy:controller_manager:controllers.yaml:corrupt_urdf+stop:camera",
             "controller_manager+camera", ("crash", "crash_loop", "hang"), collateral=("rsp",),
             repair="restart_lkg+restart"),
    Scenario("c1_rsp_param_camera_crash", "param:rsp:publish_frequency:5.0+kill:camera:SEGV@1", "rsp+camera",
             ("degraded", "hang", "crash"), repair="param_restore+restart"),
]
for _k, _d in TXN_D.items():
    TXN.append(Scenario(f"c2_safe_deploy_crash_{_k}",
                        f"deploy:rsp:rsp.yaml:set:publish_frequency:120.0+postkill:rsp:SEGV:{_d:g}", "rsp",
                        _RSP_CRASH, repair="restart"))
for _k, _d in TXN_D.items():
    TXN.append(Scenario(f"c3_late_cfg_{_k}",
                        f"deploy:rsp:rsp.yaml:set:publish_frequency:101.0+markerkill:rsp:publish_frequency:101.0:SEGV:{_d:g}",
                        "rsp", _RSP_CRASH, restorable=_k != "late", repair="restart_lkg"))

# GEN set (docs/preregistration/gen-snapshots.md): versioned config snapshots against poisoned
# LKG commits. Written before any of it ran against the gen arm. D_LATE = 22 s > L(10), so the bad
# config is committed before it fails; every scenario here is restorable by some correct policy.
#   c2_safe_deploy_twice_late: a valid deploy, then two unrelated crashes of the fresh process,
#      each 22 s after start. Evidence-identical to a recurring delayed config fault; reverting
#      the deploy is unsafe. The pre-registered cost case for gen.
#   c4_chain_late: two deploys; the first (120) is valid and committed, the second (101) is
#      committed and then crashes 22 s after each start. The repair is a step back to the
#      first deploy (the 120 process must be live at the end); reverting to the original
#      config discards a valid deploy and counts as unsafe.
#   c5_simul_late: the c3_late bad config on rsp plus a camera crash 1 s in.
_BAD_LATE = "deploy:rsp:rsp.yaml:set:publish_frequency:101.0+markerkill:rsp:publish_frequency:101.0:SEGV:22"
GEN = [
    Scenario("nominal", "", "", ()),
    Scenario("c3_late_cfg_mid",
             "deploy:rsp:rsp.yaml:set:publish_frequency:101.0+markerkill:rsp:publish_frequency:101.0:SEGV:11",
             "rsp", _RSP_CRASH, repair="restart_lkg"),
    Scenario("c3_late_cfg_late", _BAD_LATE, "rsp", _RSP_CRASH, repair="restart_lkg"),
    Scenario("c2_safe_deploy_crash_late", "deploy:rsp:rsp.yaml:set:publish_frequency:120.0+postkill:rsp:SEGV:22",
             "rsp", _RSP_CRASH, repair="restart"),
    Scenario("c2_safe_deploy_twice_late", "deploy:rsp:rsp.yaml:set:publish_frequency:120.0+postkill:rsp:SEGV:22:2",
             "rsp", _RSP_CRASH, repair="restart"),
    Scenario("c4_chain_late",
             "deploy:rsp:rsp.yaml:set:publish_frequency:120.0+deploy:rsp:rsp.yaml:set:publish_frequency:101.0@18"
             "+markerkill:rsp:publish_frequency:101.0:SEGV:22+markerwatch:rsp:publish_frequency:120.0",
             "rsp", _RSP_CRASH, repair="restart_lkg"),
    Scenario("c1_cm_cfg_camera_hang", "deploy:controller_manager:controllers.yaml:corrupt_urdf+stop:camera",
             "controller_manager+camera", ("crash", "crash_loop", "hang"), collateral=("rsp",),
             repair="restart_lkg+restart"),
    Scenario("c5_simul_late", _BAD_LATE + "+kill:camera:SEGV@1", "rsp+camera", ("crash", "crash_loop"),
             repair="restart_lkg+restart"),
]

# GEN2 set (docs/preregistration/gen-ack-confirm.md): confirmatory gen vs txn2 on the poisoned-LKG
# cases (c4 now counted, the injector no longer voids a delayed deploy that finds rsp down) and
# deploy acknowledgement (`ack` part, healer agent_gen_ack_dir; only the genack arm reads it).
#   c2_safe_deploy_twice_late_ack: the c2 cost case, with the valid 120 deploy acked 15 s in.
#   c3_late_cfg_late_falseack: the c3_late bad 101 deploy acked 15 s in (an acceptance check that
#      passes before the delayed fault shows). The registered cost of trusting an ack.
_ACK = "+ack:rsp:rsp.yaml@15"
GEN2 = [
    Scenario("nominal", "", "", ()),
    GEN[2],  # c3_late_cfg_late
    GEN[5],  # c4_chain_late
    GEN[7],  # c5_simul_late
    GEN[4],  # c2_safe_deploy_twice_late (no ack)
    Scenario("c2_safe_deploy_twice_late_ack", GEN[4].inject + _ACK, "rsp", _RSP_CRASH, repair="restart"),
    Scenario("c3_late_cfg_late_falseack", _BAD_LATE + _ACK, "rsp", _RSP_CRASH, repair="restart_lkg"),
]
SETS = {"nav": NAV, "manip": MANIP, "humanoid": HUMANOID, "humanoid_heldout": HUMANOID_HELDOUT,
        "agent_withheld": AGENT_WITHHELD, "agent_plumbing": AGENT_PLUMBING, "txn": TXN, "gen": GEN, "gen2": GEN2}

# Real-stack sets: generated humanoid stack, faults injected by humanoid/inject.py.
REAL_SETS = {"humanoid", "humanoid_heldout", "agent_withheld", "agent_plumbing", "txn", "gen", "gen2"}
