#!/usr/bin/env python3
"""Same-load A/B cost of the C++ components against their Python originals.

Five components have a one-to-one Python and C++ implementation:

    topic_rate_monitor  helix_adapter       vs helix_sensing_cpp
    anomaly_detector    helix_core          vs helix_sensing_cpp
    log_parser          helix_core          vs helix_sensing_cpp
    arbiter             helix_arbiter       vs helix_arbiter_cpp
    go2_sport_sink      helix_arbiter       vs helix_go2_sink_cpp (dry_run)

The C++ supervisor has no counterpart (it replaces heartbeat monitor,
context buffer, diagnosis and recovery), so it is not in this comparison.

Each run starts one arm's five components as plain processes (not through
``ros2 run``, whose own Python process would be sampled), then one driver
process that configures and activates the lifecycle nodes, publishes the same
load for both arms and counts what the components publish:

    /utlidar/imu 250 Hz, /utlidar/robot_odom 150 Hz, /utlidar/robot_pose 18.75 Hz,
    /utlidar/cloud 15.4 Hz (CLOUD_BYTES payload), /gnss 1 Hz, /multiplestate 1 Hz
    (rates as measured on the GO2), /rosout 10 Hz with one rule-matching line
    every 10 s, /helix_dry/nav/cmd_vel 20 Hz (vx 0.2), /lf/sportmodestate 20 Hz,
    /helix/hold 20 Hz (hold=false, as the supervisor publishes it).

The driver also counts /helix/metrics, /helix/faults and the arbiter output
(and how many of those carried the nav command), so a cheaper arm that did
less work shows up in the counts.

scripts/hw/proc_sampler.py samples the component processes. Arms alternate
(python, cpp, cpp, python, ...) so slow drift in background load on the host
does not favour one arm. Everything runs on ROS_LOCALHOST_ONLY with its own
domain id.

Usage (the unitree message workspace provides the sink's SportModeState):
    source /opt/ros/humble/setup.bash
    source <unitree_ros2>/cyclonedds_ws/install/setup.bash
    source install/setup.bash
    python3 scripts/bench_native_vs_python.py --reps 3 --seconds 120
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SAMPLER = REPO / 'scripts' / 'hw' / 'proc_sampler.py'
CLOUD_BYTES = 64 * 1024
MATCH = r'lib/helix_[a-z0-9_]+/helix_'
COMPONENTS = ['helix_topic_rate_monitor', 'helix_anomaly_detector', 'helix_log_parser',
              'helix_arbiter', 'helix_go2_sport_sink']
LIFECYCLE = COMPONENTS[:4]
PACKAGES = {
    'python': {'helix_topic_rate_monitor': 'helix_adapter',
               'helix_anomaly_detector': 'helix_core',
               'helix_log_parser': 'helix_core',
               'helix_arbiter': 'helix_arbiter',
               'helix_go2_sport_sink': 'helix_arbiter'},
    'cpp': {'helix_topic_rate_monitor': 'helix_sensing_cpp',
            'helix_anomaly_detector': 'helix_sensing_cpp',
            'helix_log_parser': 'helix_sensing_cpp',
            'helix_arbiter': 'helix_arbiter_cpp',
            'helix_go2_sport_sink': 'helix_go2_sink_cpp'},
}


def _share(pkg: str) -> Path:
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    return Path(get_package_share_directory(pkg)), Path(get_package_prefix(pkg))


def _detector_overrides() -> dict:
    """The C++ detector settings helix_native.launch.py ships."""
    share, _ = _share('helix_bringup')
    spec = importlib.util.spec_from_file_location(
        'native_launch', share / 'launch' / 'helix_native.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return dict(mod.DETECTOR_OVERRIDES)


def _param_file(tmp: Path, node: str, params: dict) -> str:
    path = tmp / f'{node}.yaml'
    import yaml
    path.write_text(yaml.safe_dump({node: {'ros__parameters': params}}))
    return str(path)


def commands(arm: str, tmp: Path) -> dict[str, list[str]]:
    """Per component: argv with the parameters the shipped launch files pass."""
    bringup, _ = _share('helix_bringup')
    arbiter_share, _ = _share('helix_arbiter')
    params = str(bringup / 'config' / 'helix_params.yaml')
    adapter = str(bringup / 'config' / 'helix_adapter_params.yaml')
    rules = str(bringup / 'config' / 'log_rules.yaml')
    arbiter = str(arbiter_share / 'config' / 'arbiter_dry.yaml')
    extra = {
        'helix_topic_rate_monitor': [adapter],
        'helix_anomaly_detector': [params],
        'helix_log_parser': [params, _param_file(tmp, 'helix_log_parser',
                                                 {'rules_file_path': rules})],
        'helix_arbiter': [arbiter],
        'helix_go2_sport_sink': [_param_file(tmp, 'helix_go2_sport_sink', {
            'mode': 'dry_run', 'input_topic': '/helix_dry/cmd_vel', 'feedback': True})],
    }
    if arm == 'cpp':
        extra['helix_topic_rate_monitor'].append(
            _param_file(tmp, 'helix_topic_rate_monitor', {'stale_after_s': 1.0}))
        extra['helix_anomaly_detector'].append(
            _param_file(tmp, 'helix_anomaly_detector', _detector_overrides()))
    out = {}
    for node, pkg in PACKAGES[arm].items():
        _, prefix = _share(pkg)
        argv = [str(prefix / 'lib' / pkg / node), '--ros-args', '-r', f'__node:={node}']
        for f in extra[node]:
            argv += ['--params-file', f]
        out[node] = argv
    return out


# --------------------------------------------------------------------------- driver


def driver(ready_file: str, counts_file: str) -> int:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Log
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import Float64MultiArray, String
    from unitree_go.msg import SportModeState

    from helix_msgs.msg import FaultEvent, HelixHold

    rclpy.init()
    node = Node('bench_load_driver')
    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    sent: dict[str, int] = {}
    got: dict[str, int] = {}

    def periodic(topic, typ, hz, qos, fill=None):
        pub = node.create_publisher(typ, topic, qos)
        msg = typ()
        if fill:
            fill(msg)
        sent[topic] = 0

        def tick():
            pub.publish(msg)
            sent[topic] += 1
        node.create_timer(1.0 / hz, tick)

    def cloud(m):
        m.height, m.width, m.point_step = 1, CLOUD_BYTES // 16, 16
        m.row_step = m.width * m.point_step
        m.data = bytes(CLOUD_BYTES)

    def nav(m):
        m.linear.x = 0.2

    periodic('/utlidar/imu', Imu, 250.0, qos_profile_sensor_data)
    periodic('/utlidar/robot_odom', Odometry, 150.0, rel)
    periodic('/utlidar/robot_pose', PoseStamped, 18.75, rel)
    periodic('/utlidar/cloud', PointCloud2, 15.4, qos_profile_sensor_data, cloud)
    periodic('/gnss', String, 1.0, rel,
             lambda m: setattr(m, 'data', '{"satellite_total":12,"satellite_inuse":9,"hdop":0.9}'))
    periodic('/multiplestate', String, 1.0, rel,
             lambda m: setattr(m, 'data', '{"volume":5,"brightness":0,"obstaclesAvoidSwitch":true,'
                                          '"uwbSwitch":false}'))
    periodic('/helix_dry/nav/cmd_vel', Twist, 20.0, rel, nav)
    periodic('/lf/sportmodestate', SportModeState, 20.0, qos_profile_sensor_data)

    # The supervisor's hold stream: without it both arbiters fail closed
    # (HELIX_STATE_MISSING) and never forward the nav command.
    hold_pub = node.create_publisher(HelixHold, '/helix/hold', rel)
    hold = HelixHold(hold=False, reason='bench', epoch=time.time_ns())
    sent['/helix/hold'] = 0

    def hold_tick():
        hold.seq += 1
        hold.stamp = time.time()
        hold_pub.publish(hold)
        sent['/helix/hold'] += 1
    node.create_timer(1.0 / 20.0, hold_tick)

    rosout = node.create_publisher(Log, '/rosout', QoSProfile(depth=100))
    sent['/rosout'] = 0
    log_i = [0]

    def log_tick():
        log_i[0] += 1
        match = log_i[0] % 100 == 0
        # The parsers act on ERROR (40) and above only.
        m = Log(name='bench_source', level=40 if match else 20)
        m.msg = ('TF extrapolation into the future requested'
                 if match else f'planner cycle {log_i[0]} ok')
        rosout.publish(m)
        sent['/rosout'] += 1
    node.create_timer(0.1, log_tick)

    counting = [False]

    def count(topic, typ, qos, weigh=lambda m: 1):
        got[topic] = 0
        got[topic + '#items'] = 0

        def cb(m):
            if counting[0]:
                got[topic] += 1
                got[topic + '#items'] += weigh(m)
        node.create_subscription(typ, topic, cb, qos)

    count('/helix/metrics', Float64MultiArray, QoSProfile(depth=100))
    count('/helix/faults', FaultEvent, QoSProfile(depth=100))
    count('/helix_dry/cmd_vel', Twist, rel, weigh=lambda m: int(m.linear.x > 0.0))

    # Lifecycle: configure then activate each node, waiting for its service.
    deadline = time.time() + 60.0
    for name in LIFECYCLE:
        cli = node.create_client(ChangeState, f'/{name}/change_state')
        while not cli.wait_for_service(timeout_sec=0.5):
            rclpy.spin_once(node, timeout_sec=0.0)
            if time.time() > deadline:
                print(f'driver: no lifecycle service for {name}', file=sys.stderr)
                return 2
        for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            fut = cli.call_async(ChangeState.Request(transition=Transition(id=tid)))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            if not fut.done() or not fut.result().success:
                print(f'driver: transition {tid} failed for {name}', file=sys.stderr)
                return 2

    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    for k in sent:
        sent[k] = 0
    counting[0] = True
    t0 = time.monotonic()
    Path(ready_file).write_text('ready\n')
    while not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.monotonic() - t0
    Path(counts_file).write_text(json.dumps({
        'elapsed_s': round(elapsed, 2),
        'sent_hz': {k: round(v / elapsed, 2) for k, v in sent.items()},
        'received': got,
        'received_hz': {k: round(v / elapsed, 3) for k, v in got.items() if '#' not in k},
    }, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0


# --------------------------------------------------------------------------- orchestration


def run_arm(arm: str, seconds: float, warmup: float, period: float, out_dir: Path,
            tag: str) -> dict:
    env = dict(os.environ, PYTHONUNBUFFERED='1')
    with tempfile.TemporaryDirectory() as tmp_s:
        tmp = Path(tmp_s)
        procs = []
        logs = []
        for name, argv in commands(arm, tmp).items():
            log = open(out_dir / f'{tag}_{name}.log', 'w')
            logs.append(log)
            procs.append(subprocess.Popen(argv, env=env, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True))
        ready, counts = tmp / 'ready', out_dir / f'{tag}_counts.json'
        drv_log = open(out_dir / f'{tag}_driver.log', 'w')
        logs.append(drv_log)
        drv = subprocess.Popen([sys.executable, __file__, '--driver', str(ready), str(counts)],
                               env=env, stdout=drv_log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        t_wait = time.time() + 90.0
        while not ready.exists():
            if drv.poll() is not None or time.time() > t_wait:
                for p in procs + [drv]:
                    p.send_signal(signal.SIGINT)
                raise RuntimeError(f'{tag}: driver did not get the components active, '
                                   f'see {out_dir}/{tag}_driver.log')
            time.sleep(0.2)
        samples = out_dir / f'{tag}_procs.jsonl'
        subprocess.run([sys.executable, str(SAMPLER), '--out', str(samples),
                        '--duration', str(seconds + warmup), '--period', str(period),
                        '--warmup', str(warmup), '--match', MATCH], check=True)
        alive = {name: p.poll() is None for name, p in zip(COMPONENTS, procs)}
        drv.send_signal(signal.SIGINT)
        drv.wait(timeout=20)
        for p in procs:
            p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        for log in logs:
            log.close()
    return {'arm': arm, 'tag': tag, 'alive_at_end': alive,
            'procs': summarize(samples, {p.pid: n for n, p in zip(COMPONENTS, procs)}, warmup),
            'driver': json.loads(counts.read_text())}


def summarize(samples: Path, names: dict[int, str], warmup: float) -> dict:
    """Per component mean/max CPU and RSS after warmup, keyed by the pid we started.

    proc_sampler names a Python process by its interpreter, so its own summary
    cannot tell the components apart; the pids can.
    """
    rows = [json.loads(line) for line in samples.read_text().splitlines()]
    rows = [r for r in rows if 'pid' in r]
    t0 = min(r['t'] for r in rows)
    out = {}
    for pid, name in names.items():
        mine = [r for r in rows if r['pid'] == pid and r['cpu_pct'] is not None
                and r['t'] - t0 >= warmup]
        if not mine:
            continue
        cpu = [r['cpu_pct'] for r in mine]
        rss = [r['rss_kb'] for r in mine]
        out[name] = {'n': len(mine), 'cpu_mean': round(statistics.fmean(cpu), 3),
                     'cpu_max': round(max(cpu), 3),
                     'rss_mean_kb': round(statistics.fmean(rss), 1), 'rss_max_kb': max(rss),
                     'threads': mine[-1]['threads']}
    return out


def aggregate(runs: list[dict]) -> dict:
    out = {}
    for name in COMPONENTS:
        row = {}
        for arm in ('python', 'cpp'):
            cpu = [r['procs'][name]['cpu_mean'] for r in runs
                   if r['arm'] == arm and name in r['procs']]
            rss = [r['procs'][name]['rss_mean_kb'] / 1024.0 for r in runs
                   if r['arm'] == arm and name in r['procs']]
            row[arm] = {'runs': len(cpu),
                        'cpu_pct_mean': round(statistics.fmean(cpu), 3) if cpu else None,
                        'cpu_pct_per_run': [round(c, 3) for c in cpu],
                        'rss_mb_mean': round(statistics.fmean(rss), 2) if rss else None}
        p, c = row['python'], row['cpp']
        if p['cpu_pct_mean'] and c['cpu_pct_mean'] is not None:
            row['cpu_change_pct'] = round(100.0 * (c['cpu_pct_mean'] / p['cpu_pct_mean'] - 1), 1)
            row['rss_change_pct'] = round(100.0 * (c['rss_mb_mean'] / p['rss_mb_mean'] - 1), 1)
        out[name] = row
    for arm in ('python', 'cpp'):
        rs = [r for r in runs if r['arm'] == arm]
        out.setdefault('total', {})[arm] = {
            'cpu_pct_mean': round(statistics.fmean(
                sum(r['procs'][n]['cpu_mean'] for n in COMPONENTS if n in r['procs'])
                for r in rs), 3),
            'rss_mb_mean': round(statistics.fmean(
                sum(r['procs'][n]['rss_mean_kb'] for n in COMPONENTS if n in r['procs']) / 1024.0
                for r in rs), 2),
        }
    return out


def main() -> int:
    if len(sys.argv) == 4 and sys.argv[1] == '--driver':
        return driver(sys.argv[2], sys.argv[3])
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--seconds', type=float, default=120.0)
    ap.add_argument('--warmup', type=float, default=20.0)
    ap.add_argument('--period', type=float, default=2.0)
    ap.add_argument('--domain', type=int, default=87)
    ap.add_argument('--out', default=str(REPO / 'results' / 'native_vs_python_bench.json'))
    ap.add_argument('--raw-dir', default=None,
                    help='per-run logs and samples (default: a new temp dir)')
    args = ap.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    raw = Path(args.raw_dir or tempfile.mkdtemp(prefix='helix_ab_'))
    raw.mkdir(parents=True, exist_ok=True)
    order = []
    for i in range(args.reps):
        order += ['python', 'cpp'] if i % 2 == 0 else ['cpp', 'python']
    runs = []
    for i, arm in enumerate(order):
        tag = f'{i:02d}_{arm}'
        print(f'{datetime.now(timezone.utc):%H:%M:%SZ} run {tag}', flush=True)
        runs.append(run_arm(arm, args.seconds, args.warmup, args.period, raw, tag))
        time.sleep(5.0)  # let the previous arm's DDS participants expire
        print(json.dumps({n: (v['cpu_mean'], round(v['rss_mean_kb'] / 1024, 1))
                          for n, v in runs[-1]['procs'].items()}), flush=True)
    sha = subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--short', 'HEAD'],
                         capture_output=True, text=True).stdout.strip()
    result = {
        'generated_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'git_sha': sha, 'host': f'{platform.machine()} host', 'cpus': os.cpu_count(),
        'loadavg_end': os.getloadavg(),
        'rmw': os.environ['RMW_IMPLEMENTATION'],
        'seconds_per_run': args.seconds, 'warmup_s': args.warmup, 'period_s': args.period,
        'cloud_bytes': CLOUD_BYTES, 'order': order,
        'cpu_unit': 'percent of one core, mean of per-sample deltas after warmup',
        'aggregate': aggregate(runs), 'runs': runs,
    }
    Path(args.out).write_text(json.dumps(result, indent=1) + '\n')
    print(json.dumps(result['aggregate'], indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
