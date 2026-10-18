#!/usr/bin/env python3
"""Finite non-actuating GO2 inspection, never a motion or mode-control tool.

Default: subscriptions and /proc sampling only. --query-metadata additionally
sends the fixed read-only RPCs in QUERIES once each. No arbitrary API, service,
topic or payload argument exists. Protocol versions are not firmware versions.
See docs/CONTROLLER_INSPECTION.md. Raw output is local evidence to review before
publication; serial numbers, addresses, credentials and command lines are omitted.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import platform
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

# Official unitree_sdk2 d1abf001: internal_api.hpp, robot_state_api.hpp;
# unitree_sdk2_python 814556d1: comm/motion_switcher/motion_switcher_api.py.
# IDs are scoped to the service. Sport 1001 is Damp and is NEVER sent here.
QUERIES = (
    ('sport_api_version', 'sport', 1),
    ('robot_state_api_version', 'robot_state', 1),
    ('motion_switcher_api_version', 'motion_switcher', 1),
    ('service_list', 'robot_state', 1003),
    ('check_mode', 'motion_switcher', 1001),
)
NATIVE = ('helix_topic_rate_monitor', 'helix_anomaly_detector', 'helix_log_parser',
          'helix_supervisor', 'helix_arbiter', 'helix_go2_sport_sink')


def query_payload(name):
    """Only the pinned allowlist can become a request."""
    return next((service, api, '{}') for key, service, api in QUERIES if key == name)


def metadata(name, text):
    """Extract the documented fields, never retain arbitrary response contents."""
    if len(text) > 32768:
        raise ValueError('oversize metadata')
    try:
        value = json.loads(text)
    except ValueError:
        value = text
    if name.endswith('_api_version'):
        version = value.get('version') if isinstance(value, dict) else value
        if not isinstance(version, str) or not version or len(version) > 128:
            raise ValueError('unexpected version schema')
        return {'server_api_version': version, 'firmware_build': None}
    if name == 'check_mode' and isinstance(value, dict):
        if all(isinstance(value.get(k), str) and len(value[k]) <= 128 for k in ('form', 'name')):
            return {k: value[k] for k in ('form', 'name')}
    if name == 'service_list' and isinstance(value, list):
        if all(isinstance(row, dict) and isinstance(row.get('name'), str) and
               len(row['name']) <= 128 and type(row.get('status')) is int and
               type(row.get('protect')) in (int, bool) and row['protect'] in (0, 1)
               for row in value):
            return [{k: row[k] for k in ('name', 'status', 'protect')} for row in value]
    raise ValueError('unexpected metadata schema')


def process_snapshot():
    result = {}
    domain = os.environ.get('ROS_DOMAIN_ID', '0').encode()
    for p in Path('/proc').iterdir():
        if not p.name.isdecimal():
            continue
        try:
            name = (p / 'exe').resolve().name
            if name not in NATIVE:
                continue
            process_env = (p / 'environ').read_bytes().split(b'\0')
            actual_domain = next((s.split(b'=', 1)[1] for s in process_env
                                  if s.startswith(b'ROS_DOMAIN_ID=')), b'0')
            if actual_domain != domain:
                continue
            raw = (p / 'stat').read_text()
            fields = raw[raw.rindex(')') + 2:].split()
            status = (p / 'status').read_text().splitlines()
            rss = next(int(s.split()[1]) for s in status if s.startswith('VmRSS:'))
            result[(int(p.name), int(fields[19]))] = dict(
                name=name, ticks=int(fields[11]) + int(fields[12]),
                rss_kb=rss, threads=int(fields[17]))
        except (OSError, ValueError, StopIteration):
            continue
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--warmup', type=float, default=10)
    parser.add_argument('--query-metadata', action='store_true')
    parser.add_argument('--evidence-class', choices=('HW-passive', 'Sim'), required=True,
                        help='operator labels the source; the tool cannot authenticate a robot')
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.seconds, args.warmup)) or not 0 <= args.warmup < args.seconds <= 3600:
        parser.error('requires 0 <= warmup < seconds <= 3600')
    if args.query_metadata and args.seconds < 10:
        parser.error('metadata inspection requires at least ten seconds')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from unitree_api.msg import Request, Response
    from unitree_go.msg import LowState, SportModeState

    from helix_msgs.msg import ArbiterStatus, SinkStatus, SupervisorStatus

    rclpy.init(args=[])
    node = rclpy.create_node('helix_controller_inspect')
    started = time.monotonic()
    utc = datetime.now(timezone.utc).isoformat()
    streams = {}
    latest = {}
    violations = collections.Counter()
    observed_apis = collections.Counter()
    queries, pending, publishers = {}, {}, {}
    resources = collections.defaultdict(list)
    aggregate = []
    thermal_peak_c = None
    previous, previous_at = {}, None

    def received(topic):
        now = time.monotonic()
        s = streams.setdefault(topic, dict(count=0, first=now, last=now, max_gap_s=0))
        if s['count']:
            s['max_gap_s'] = max(s['max_gap_s'], now - s['last'])
        s.update(count=s['count'] + 1, last=now)

    def low(m):
        received('/lowstate')
        latest['lowstate_version_raw'] = [int(x) for x in m.version]
        # Deliberately exclude sn, motor serials and remote button bytes.

    def sport(m):
        received('/lf/sportmodestate')
        latest['sport_state'] = dict(mode=int(m.mode), gait_type=int(m.gait_type),
                                   speed_xy=math.hypot(*m.velocity[:2]) if all(math.isfinite(v) for v in m.velocity[:2]) else None,
                                   yaw_speed=float(m.yaw_speed) if math.isfinite(m.yaw_speed) else None)

    def arbiter(m):
        received('/helix/arbiter/status')
        latest['arbiter_protective_latched'] = bool(m.protective_latched)
        if not m.protective_latched:
            violations['arbiter_released'] += 1
        if any(abs(x) > 1e-9 or not math.isfinite(x)
               for x in (m.out_linear_x, m.out_linear_y, m.out_angular_z)):
            violations['nonzero_arbiter_output'] += 1

    def sink(m):
        received('/helix/sink/status')
        latest['sink_mode'] = m.mode
        if m.mode != 'dry_run':
            violations['sink_not_dry_run'] += 1

    def supervisor(m):
        received('/helix/supervisor/status')
        latest['supervisor_state'] = m.state
        latest['supervisor_missing_nodes'] = list(m.missing_nodes)
        if m.actuation_enabled:
            violations['supervisor_actuation_enabled'] += 1

    def response(service, m):
        key = (service, m.header.identity.id, m.header.identity.api_id)
        if key not in pending:
            return
        name, sent = pending.pop(key)
        if time.monotonic() - sent > 2:
            queries[name] = dict(error='late_response')
            return
        result = dict(code=int(m.header.status.code), latency_ms=1000 * (time.monotonic() - sent))
        if result['code'] == 0:
            try:
                result['metadata'] = metadata(name, m.data)
            except ValueError as e:
                result['schema_error'] = str(e)
        queries[name] = result

    for cls, topic, callback in (
        (LowState, '/lowstate', low), (SportModeState, '/lf/sportmodestate', sport),
    ):
        node.create_subscription(cls, topic, callback, qos_profile_sensor_data)
    for cls, topic, callback in (
        (ArbiterStatus, '/helix/arbiter/status', arbiter),
        (SinkStatus, '/helix/sink/status', sink),
        (SupervisorStatus, '/helix/supervisor/status', supervisor),
    ):
        node.create_subscription(cls, topic, callback, 100)
    node.create_subscription(Request, '/api/sport/request', lambda m:
                             observed_apis.update([int(m.header.identity.api_id)]), 100)
    if args.query_metadata:
        for service in {q[1] for q in QUERIES}:
            publishers[service] = node.create_publisher(Request, f'/api/{service}/request', 10)
            if publishers[service].topic_name != f'/api/{service}/request':
                raise RuntimeError('metadata request topic was remapped')
            node.create_subscription(Response, f'/api/{service}/response',
                                     lambda m, service=service: response(service, m), 100)
    queue = list(QUERIES) if args.query_metadata else []
    next_query = started + 3
    next_sample = started
    sent_apis = []
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if queue and now >= next_query:
                name, service, api = queue.pop(0)
                assert query_payload(name) == (service, api, '{}')
                message = Request()
                message.header.identity.id = time.monotonic_ns()
                message.header.identity.api_id = api
                message.header.policy.noreply = False
                message.parameter = '{}'
                pending[(service, message.header.identity.id, api)] = (name, now)
                publishers[service].publish(message)
                sent_apis.append(dict(service=service, api=api))
                next_query = now + .5
            for key, (name, sent) in list(pending.items()):
                if now - sent > 2:
                    queries[name] = dict(error='response_timeout')
                    del pending[key]
            if now >= next_sample:
                snapshot = process_snapshot()
                interval = []
                if previous_at is not None and now - started >= args.warmup:
                    for key, sample in snapshot.items():
                        if key in previous:
                            cpu = 100 * (sample['ticks'] - previous[key]['ticks']) / os.sysconf('SC_CLK_TCK') / (now - previous_at)
                            resources[sample['name']].append(dict(cpu_pct=cpu, rss_kb=sample['rss_kb'], threads=sample['threads']))
                            interval.append(dict(cpu_pct=cpu, rss_kb=sample['rss_kb']))
                    if len(interval) == len(NATIVE) and {s['name'] for s in snapshot.values()} == set(NATIVE):
                        aggregate.append(dict(cpu_pct=sum(r['cpu_pct'] for r in interval),
                                              rss_kb=sum(r['rss_kb'] for r in interval)))
                for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
                    try:
                        value = float(path.read_text())/1000
                        if -20 < value < 150:
                            thermal_peak_c = value if thermal_peak_c is None else max(thermal_peak_c, value)
                    except (OSError, ValueError):
                        pass
                previous, previous_at = snapshot, now
                next_sample = now + 1
    finally:
        ended = time.monotonic()
        own_publishers = [p.topic_name for p in node.publishers]
        # Evidence includes only this node's publisher names, no host identities.
        node.destroy_node()
        rclpy.shutdown()
    summary = {}
    def summarize(rows):
        cpu = sorted(r['cpu_pct'] for r in rows)
        return dict(samples=len(rows), cpu_mean=statistics.fmean(cpu),
                    cpu_p95=cpu[max(0, math.ceil(.95*len(cpu))-1)], cpu_max=max(cpu),
                    rss_mean_kb=statistics.fmean(r['rss_kb'] for r in rows),
                    rss_max_kb=max(r['rss_kb'] for r in rows))
    for name, rows in resources.items():
        summary[name] = summarize(rows)
    for name, _, _ in queue:
        queries[name] = dict(error='not_sent_before_end')
    for name, _ in pending.values():
        queries[name] = dict(error='observation_ended_before_response')
    topics = {name: dict(count=s['count'], rate_hz=(s['count']-1)/(s['last']-s['first'])
                        if s['count'] > 1 else None, max_gap_s=s['max_gap_s'],
                        age_at_end_s=ended-s['last']) for name, s in streams.items()}
    process_counts = dict(collections.Counter(s['name'] for s in previous.values()))
    result = dict(evidence_class=args.evidence_class, utc=utc, duration_s=ended-started,
                  architecture=platform.machine(), kernel_release=platform.release(),
                  queries=queries, requests_sent=sent_apis, own_publishers=own_publishers,
                  topics=topics, observed_sport_api_counts=dict(observed_apis),
                  latest=latest, nonactuating_profile_violations=dict(violations), resources=summary,
                  aggregate_resources=summarize(aggregate) if aggregate else None,
                  thermal_peak_c=thermal_peak_c,
                  native_process_counts=process_counts,
                  missing_native_processes=sorted(set(NATIVE)-set(summary)),
                  native_validation_complete=(not violations and set(NATIVE) <= set(summary) and
                      all(process_counts.get(name) == 1 for name in NATIVE) and
                      all(name in topics and topics[name]['age_at_end_s'] < .5 for name in (
                          '/lowstate', '/lf/sportmodestate', '/helix/arbiter/status',
                          '/helix/supervisor/status', '/helix/sink/status')) and
                      latest.get('supervisor_state') == 'NOMINAL'),
                  firmware_build=None, controller_motion_timeout_s=None,
                  motion_timeout_measured=False,
                  notes=['API version and raw LowState.version do not identify the firmware build.',
                         'Resource samples include exact native executable names in this ROS domain, excluding this observer.',
                         'No command-silence motion test or stop mechanism test is performed.'])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print('Inspection saved; no motion or mode-change API is implemented.')
    return 1 if violations else 0


if __name__ == '__main__':
    raise SystemExit(main())
