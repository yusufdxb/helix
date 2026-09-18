"""Hardware preflight: verifies the motion graph WITHOUT commanding motion.

Publishes nothing. Collects a snapshot of the live ROS graph (nodes, topic
types, endpoint QoS, lifecycle states, GO2 state freshness, clocks, arbiter
status and HELIX hold state), then evaluates it with the pure function
``evaluate`` and prints GO or NO-GO. Any FAIL is NO-GO. WARN never blocks but
is printed and recorded.

  ros2 run helix_arbiter helix_preflight --stage B --out preflight.json \
      [--sport-baseline stageA_sport_publishers.json]
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional

PASS, WARN, FAIL = 'PASS', 'WARN', 'FAIL'

ARBITER = '/helix_arbiter'
RECOVERY = '/helix_recovery_node'
DIAGNOSIS = '/helix_diagnosis_node'
SINK = '/helix_go2_sport_sink'
OBSERVERS = ('/helix_trace_recorder', '/helix_preflight', '/helix_hw_stage',
             '/helix_harness')

T_TWIST = 'geometry_msgs/msg/Twist'
T_HOLD = 'helix_msgs/msg/HelixHold'
T_STATUS = 'helix_msgs/msg/ArbiterStatus'
T_REQUEST = 'unitree_api/msg/Request'
T_ODOM = 'nav_msgs/msg/Odometry'

# Sink mode each stage expects. None = sink may be absent.
STAGE_SINK_MODE = {'A': 'dry_run', 'B': 'stop_only', 'C': 'dry_run',
                   'D': 'armed', 'E': 'armed', 'F': 'armed'}


@dataclass
class Check:
    id: str
    name: str
    result: str
    detail: str


@dataclass
class Config:
    stage: str = 'A'
    output_topic: str = '/cmd_vel'
    hold_topic: str = '/helix/hold'
    status_topic: str = '/helix/arbiter/status'
    request_topic: str = '/api/sport/request'
    source_topics: tuple = ('/teleop/cmd_vel', '/nav/cmd_vel')
    state_topic: str = '/utlidar/robot_odom'
    min_state_hz: float = 50.0
    max_state_age_s: float = 0.2
    max_hold_age_s: float = 0.5
    max_robot_clock_skew_s: float = 5.0
    sport_baseline: Optional[List[str]] = None
    rehearsal: bool = False      # off-robot rehearsal against fake_go2: waives C8b only


def _names(endpoints) -> List[str]:
    return sorted({e['node'] for e in endpoints})


def _qos_ok(pub: dict, sub: dict) -> Optional[str]:
    """DDS request/offer rules for the two policies that bite in practice."""
    if sub['reliability'] == 'RELIABLE' and pub['reliability'] == 'BEST_EFFORT':
        return f"{pub['node']} BEST_EFFORT -> {sub['node']} RELIABLE"
    if sub['durability'] == 'TRANSIENT_LOCAL' and pub['durability'] == 'VOLATILE':
        return f"{pub['node']} VOLATILE -> {sub['node']} TRANSIENT_LOCAL"
    return None


def evaluate(snap: dict, cfg: Config) -> List[Check]:
    c: List[Check] = []
    nodes = set(snap.get('nodes', []))
    topics: Dict[str, dict] = snap.get('topics', {})
    life = snap.get('lifecycle', {})
    sink_mode_expected = STAGE_SINK_MODE[cfg.stage]

    def topic(t):
        return topics.get(t, {'types': [], 'publishers': [], 'subscribers': []})

    def add(i, name, ok, detail, warn=False):
        c.append(Check(i, name, PASS if ok else (WARN if warn else FAIL), detail))

    # C1 expected nodes
    need = [ARBITER, RECOVERY, DIAGNOSIS, SINK]
    missing = [n for n in need if n not in nodes]
    add('C1', 'expected HELIX nodes present', not missing,
        f'missing {missing}' if missing else f'{need}')

    # C2 lifecycle
    bad = {n: life.get(n) for n in (ARBITER, RECOVERY, DIAGNOSIS) if life.get(n) != 'active'}
    add('C2', 'arbiter, recovery, diagnosis lifecycle active', not bad,
        f'not active: {bad}' if bad else 'all active')

    # C3 topic types
    want = {cfg.output_topic: T_TWIST, cfg.hold_topic: T_HOLD,
            cfg.status_topic: T_STATUS, cfg.state_topic: T_ODOM}
    for t in cfg.source_topics:
        if topic(t)['types']:
            want[t] = T_TWIST
    if sink_mode_expected != 'dry_run':
        want[cfg.request_topic] = T_REQUEST
    wrong = {t: topic(t)['types'] for t, ty in want.items() if topic(t)['types'] != [ty]}
    add('C3', 'topic types', not wrong, f'wrong/missing: {wrong}' if wrong else 'ok')

    # C4 QoS compatibility on every motion-path edge
    issues = []
    for t in (cfg.output_topic, cfg.hold_topic, *cfg.source_topics):
        tp = topic(t)
        for p in tp['publishers']:
            for s in tp['subscribers']:
                why = _qos_ok(p, s)
                if why:
                    issues.append(f'{t}: {why}')
    add('C4', 'QoS compatible on motion edges', not issues, '; '.join(issues) or 'ok')

    # C5 exactly one publisher on the authoritative output: the arbiter
    pubs = _names(topic(cfg.output_topic)['publishers'])
    add('C5', f'single authoritative publisher on {cfg.output_topic}', pubs == [ARBITER],
        f'publishers={pubs}')

    # C6 final command sink: exactly the GO2 sport sink consumes the output
    subs = [n for n in _names(topic(cfg.output_topic)['subscribers']) if n not in OBSERVERS
            and not n.startswith('/rosbag2')]
    add('C6', f'final sink on {cfg.output_topic}', subs == [SINK], f'consumers={subs}')

    # C7 sink mode matches the stage
    mode = snap.get('sink_mode')
    add('C7', f'sink mode == {sink_mode_expected}', mode == sink_mode_expected,
        f'sink mode={mode}')

    # C8 no competing motion authority
    comp = []
    if '/twist_mux' in nodes:
        comp.append('twist_mux running (second mux, goes silent on idle/lock)')
    legacy = topic('/helix/cmd_vel')
    if legacy['publishers'] or legacy['subscribers']:
        comp.append(f"/helix/cmd_vel in use pubs={_names(legacy['publishers'])} "
                    f"subs={_names(legacy['subscribers'])}")
    for t in cfg.source_topics:
        if ARBITER not in _names(topic(t)['subscribers']) and topic(t)['publishers']:
            comp.append(f'{t} has publishers but the arbiter is not subscribed')
    req_pubs = _names(topic(cfg.request_topic)['publishers'])
    helix_req = [n for n in req_pubs if n.startswith('/helix')]
    if sink_mode_expected == 'dry_run':
        if helix_req:
            comp.append(f'HELIX node publishing {cfg.request_topic} in dry_run: {helix_req}')
    elif helix_req != [SINK]:
        comp.append(f'HELIX publishers on {cfg.request_topic} = {helix_req}, want [{SINK}]')
    if cfg.sport_baseline is not None:
        extra = [n for n in req_pubs if n not in cfg.sport_baseline and n != SINK]
        if extra:
            comp.append(f'publishers on {cfg.request_topic} not in stage-A baseline: {extra}')
    add('C8', 'no competing motion publisher', not comp, '; '.join(comp) or 'ok')
    if cfg.sport_baseline is None and cfg.stage != 'A' and not cfg.rehearsal:
        add('C8b', 'stage-A sport publisher baseline supplied', False,
            'pass --sport-baseline from stage A; competing robot-side publishers unverifiable')

    # C13 single HELIX state authority: a second publisher could release a hold
    hp = _names(topic(cfg.hold_topic)['publishers'])
    hint_p = _names(topic('/helix/recovery_hints')['publishers'])
    add('C13', 'hold/hint topics have exactly one HELIX publisher',
        hp == [RECOVERY] and hint_p == [DIAGNOSIS],
        f'{cfg.hold_topic} pubs={hp}; /helix/recovery_hints pubs={hint_p}')

    # C9 GO2 state fresh
    st = snap.get('state', {})
    hz, age = st.get('rate_hz', 0.0), st.get('last_age_s', math.inf)
    # Stage A is motors-off: LiDAR odometry may legitimately idle, so WARN.
    add('C9', f'GO2 state fresh on {cfg.state_topic}',
        hz >= cfg.min_state_hz and age <= cfg.max_state_age_s,
        f'rate={hz:.1f} Hz (min {cfg.min_state_hz}), age={age:.3f}s',
        warn=cfg.stage == 'A')

    # C10 clock sanity
    clk = snap.get('clock', {})
    wall, commit = clk.get('wall_now', 0.0), clk.get('git_commit_time', 0.0)
    add('C10', 'local clock sane (after HEAD commit time, not 1970)',
        wall > commit > 0 and dt.datetime.utcfromtimestamp(wall).year >= 2026,
        f'wall={dt.datetime.utcfromtimestamp(wall).isoformat()}Z '
        f'commit={dt.datetime.utcfromtimestamp(commit).isoformat()}Z')
    skew = clk.get('robot_skew_s')
    add('C10b', 'robot header clock skew recorded', skew is not None
        and abs(skew) <= cfg.max_robot_clock_skew_s,
        f'robot_stamp - local = {skew} s (freshness uses receipt time, so skew '
        f'does not affect safety; recorded for evidence)', warn=True)

    # C11 HELIX hold state live
    hold = snap.get('hold', {})
    add('C11', 'HELIX hold state fresh', hold.get('age_s', math.inf) <= cfg.max_hold_age_s,
        f"age={hold.get('age_s')} hold={hold.get('hold')} fault={hold.get('fault_id')!r}")

    # C11b a stage that commands must not start inside a hold: it would be
    # confounded (idle GO2 baselines produce spurious HELIX anomalies).
    add('C11b', 'HELIX not holding at stage start', hold.get('hold') is False,
        f"hold={hold.get('hold')} fault={hold.get('fault_id')!r}", warn=cfg.stage == 'A')

    # C12 arbiter selection report
    s = snap.get('status', {})
    ok_reason = s.get('reason') in ('NO_LIVE_INPUT', 'SOURCE', 'HELIX_HOLD')
    zero = s.get('out_linear_x') == 0.0 and s.get('out_angular_z') == 0.0
    add('C12', 'arbiter selected command is zero at preflight', ok_reason and zero,
        f"reason={s.get('reason')} source={s.get('selected_source')!r} "
        f"out=({s.get('out_linear_x')},{s.get('out_angular_z')}) "
        f"sink_subscribers={s.get('sink_subscribers')}")
    return c


def verdict(checks: List[Check]) -> str:
    return 'NO-GO' if any(k.result == FAIL for k in checks) else 'GO'


# --------------------------------------------------------------------------
# collector (ROS)
# --------------------------------------------------------------------------

def _qos_dict(ep) -> dict:
    q = ep.qos_profile
    return {'node': (ep.node_namespace.rstrip('/') + '/' + ep.node_name),
            'reliability': q.reliability.name, 'durability': q.durability.name}


def git_info(repo: Optional[str] = None) -> Dict[str, object]:
    def g(*a):
        return subprocess.run(['git', *a], capture_output=True, text=True,
                              cwd=repo).stdout.strip()
    return {'sha': g('rev-parse', 'HEAD'), 'dirty': bool(g('status', '--porcelain')),
            'commit_time': float(g('show', '-s', '--format=%ct', 'HEAD') or 0)}


def collect(node, cfg: Config, listen_s: float = 2.0, repo: Optional[str] = None) -> dict:
    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    from helix_msgs.msg import ArbiterStatus, HelixHold

    snap: dict = {'taken_wall': time.time()}
    # Let discovery settle before reading the graph.
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    snap['nodes'] = sorted((ns.rstrip('/') + '/' + n)
                           for n, ns in node.get_node_names_and_namespaces())
    topics = {}
    for name, types in node.get_topic_names_and_types():
        topics[name] = {
            'types': list(types),
            'publishers': [_qos_dict(e) for e in node.get_publishers_info_by_topic(name)],
            'subscribers': [_qos_dict(e) for e in node.get_subscriptions_info_by_topic(name)],
        }
    snap['topics'] = topics

    life = {}
    for n in (ARBITER, RECOVERY, DIAGNOSIS):
        cli = node.create_client(GetState, f'{n}/get_state')
        if cli.wait_for_service(timeout_sec=2.0):
            fut = cli.call_async(GetState.Request())
            end = time.monotonic() + 2.0
            while not fut.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if fut.done() and fut.result() is not None:
                life[n] = fut.result().current_state.label
        node.destroy_client(cli)
    snap['lifecycle'] = life

    rx = {'odom': [], 'status': None, 'hold': None, 'hold_t': None, 'sink_mode': None}

    def on_odom(m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rx['odom'].append((time.monotonic(), time.time(), stamp))

    def on_hold(m):
        rx['hold'], rx['hold_t'] = m, time.monotonic()

    def on_sink(m):
        try:
            rx['sink_mode'] = json.loads(m.data).get('mode')
        except ValueError:
            pass
    subs = [node.create_subscription(Odometry, cfg.state_topic, on_odom, qos_profile_sensor_data),
            node.create_subscription(ArbiterStatus, cfg.status_topic,
                                     lambda m: rx.__setitem__('status', m), 10),
            node.create_subscription(HelixHold, cfg.hold_topic, on_hold, 10),
            node.create_subscription(String, '/helix/sink/trace', on_sink, 10)]
    end = time.monotonic() + listen_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    now = time.monotonic()
    od = rx['odom']
    snap['state'] = {
        'count': len(od),
        'rate_hz': (len(od) - 1) / (od[-1][0] - od[0][0]) if len(od) > 2 else 0.0,
        'last_age_s': now - od[-1][0] if od else math.inf}
    s = rx['status']
    snap['status'] = {} if s is None else {
        'reason': s.reason, 'selected_source': s.selected_source,
        'out_linear_x': s.out_linear_x, 'out_angular_z': s.out_angular_z,
        'hold_active': s.hold_active, 'sink_subscribers': s.sink_subscribers,
        'rejected_total': s.rejected_total}
    h = rx['hold']
    snap['hold'] = {} if h is None else {
        'hold': h.hold, 'fault_id': h.fault_id, 'age_s': now - rx['hold_t']}
    snap['sink_mode'] = rx['sink_mode'] or _sink_mode_param(node)
    gi = git_info(repo)
    snap['git'] = gi
    snap['clock'] = {'wall_now': time.time(), 'git_commit_time': gi['commit_time'],
                     'robot_skew_s': (round(od[-1][2] - od[-1][1], 3) if od else None)}
    for sub in subs:
        node.destroy_subscription(sub)
    return snap


def _sink_mode_param(node) -> Optional[str]:
    """Fallback when the sink has not traced yet: read its mode parameter."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, f'{SINK}/get_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=1.0):
            return None
        fut = cli.call_async(GetParameters.Request(names=['mode']))
        end = time.monotonic() + 2.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        vals = fut.result().values if fut.done() and fut.result() else []
        return vals[0].string_value if vals else None
    finally:
        node.destroy_client(cli)


def run(stage: str, out: Optional[str], baseline: Optional[str], rehearsal: bool,
        node=None, repo: Optional[str] = None) -> dict:
    import rclpy
    own = node is None
    if own:
        rclpy.init()
        node = rclpy.create_node('helix_preflight')
    try:
        cfg = Config(stage=stage, rehearsal=rehearsal,
                     sport_baseline=(json.load(open(baseline)) if baseline else None))
        snap = collect(node, cfg, repo=repo)
        checks = evaluate(snap, cfg)
    finally:
        if own:
            node.destroy_node()
            rclpy.shutdown()
    report = {'stage': stage, 'rehearsal': rehearsal, 'verdict': verdict(checks),
              'checks': [asdict(k) for k in checks], 'snapshot': snap}
    if out:
        with open(out, 'w') as fp:
            json.dump(report, fp, indent=1, default=str)
    return report


def print_report(rep: dict) -> None:
    for k in rep['checks']:
        print(f"[{k['result']:4}] {k['id']:4} {k['name']}: {k['detail']}")
    tag = ' (REHEARSAL, not hardware)' if rep['rehearsal'] else ''
    print(f"\nSTAGE {rep['stage']} PREFLIGHT: {rep['verdict']}{tag}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='HELIX motion-path hardware preflight (no motion)')
    ap.add_argument('--stage', choices=sorted(STAGE_SINK_MODE), default='A')
    ap.add_argument('--out')
    ap.add_argument('--sport-baseline', help='JSON list of /api/sport/request publishers '
                    'captured at stage A before HELIX was launched')
    ap.add_argument('--capture-sport-baseline', metavar='FILE',
                    help='write the current /api/sport/request publisher list and exit')
    ap.add_argument('--rehearsal', action='store_true')
    a = ap.parse_args(argv)
    if a.capture_sport_baseline:
        import rclpy
        rclpy.init()
        n = rclpy.create_node('helix_preflight')
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        pubs = sorted({_qos_dict(e)['node'] for e in
                       n.get_publishers_info_by_topic('/api/sport/request')})
        json.dump(pubs, open(a.capture_sport_baseline, 'w'), indent=1)
        print(f'{len(pubs)} publishers on /api/sport/request: {pubs}')
        n.destroy_node()
        rclpy.shutdown()
        return 0
    rep = run(a.stage, a.out, a.sport_baseline, a.rehearsal)
    print_report(rep)
    return 0 if rep['verdict'] == 'GO' else 1


if __name__ == '__main__':
    sys.exit(main())
