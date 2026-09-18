"""Pure tests of every preflight GO/NO-GO condition (synthetic graph snapshots)."""
import copy
import time

import pytest
from helix_arbiter.preflight import (
    ARBITER,
    DIAGNOSIS,
    RECOVERY,
    SINK,
    Config,
    evaluate,
    verdict,
)

R, BE, V, TL = 'RELIABLE', 'BEST_EFFORT', 'VOLATILE', 'TRANSIENT_LOCAL'


def ep(node, rel=R, dur=V):
    return {'node': node, 'reliability': rel, 'durability': dur}


def good(stage='E'):
    now = time.time()
    topics = {
        '/cmd_vel': {'types': ['geometry_msgs/msg/Twist'],
                     'publishers': [ep(ARBITER)], 'subscribers': [ep(SINK)]},
        '/helix/hold': {'types': ['helix_msgs/msg/HelixHold'],
                        'publishers': [ep(RECOVERY)], 'subscribers': [ep(ARBITER, BE)]},
        '/helix/arbiter/status': {'types': ['helix_msgs/msg/ArbiterStatus'],
                                  'publishers': [ep(ARBITER)], 'subscribers': []},
        '/nav/cmd_vel': {'types': ['geometry_msgs/msg/Twist'], 'publishers': [],
                         'subscribers': [ep(ARBITER, BE)]},
        '/teleop/cmd_vel': {'types': ['geometry_msgs/msg/Twist'], 'publishers': [],
                            'subscribers': [ep(ARBITER, BE)]},
        '/helix/recovery_hints': {'types': ['helix_msgs/msg/RecoveryHint'],
                                  'publishers': [ep(DIAGNOSIS)],
                                  'subscribers': [ep(RECOVERY)]},
        '/utlidar/robot_odom': {'types': ['nav_msgs/msg/Odometry'],
                                'publishers': [ep('/robot')], 'subscribers': []},
        '/api/sport/request': {'types': ['unitree_api/msg/Request'],
                               'publishers': [ep('/stock_a'), ep(SINK)], 'subscribers': []},
    }
    return {
        'nodes': [ARBITER, RECOVERY, DIAGNOSIS, SINK, '/stock_a'],
        'topics': topics,
        'lifecycle': {ARBITER: 'active', RECOVERY: 'active', DIAGNOSIS: 'active'},
        'sink_mode': {'A': 'dry_run', 'B': 'stop_only', 'C': 'dry_run'}.get(stage, 'armed'),
        'state': {'rate_hz': 150.0, 'last_age_s': 0.01},
        'clock': {'wall_now': now, 'git_commit_time': now - 3600, 'robot_skew_s': 0.2},
        'hold': {'hold': False, 'fault_id': '', 'age_s': 0.05},
        'status': {'reason': 'NO_LIVE_INPUT', 'selected_source': '', 'out_linear_x': 0.0,
                   'out_angular_z': 0.0, 'sink_subscribers': 1},
    }


def cfg(stage='E', **kw):
    kw.setdefault('sport_baseline', ['/stock_a'])
    return Config(stage=stage, **kw)


def failed(snap, c):
    return {k.id for k in evaluate(snap, c) if k.result == 'FAIL'}


@pytest.mark.parametrize('stage', list('BCDEF'))
def test_good_graph_is_go(stage):
    s = good(stage)
    if stage == 'C':
        s['topics']['/api/sport/request']['publishers'] = [ep('/stock_a')]
    assert verdict(evaluate(s, cfg(stage))) == 'GO', evaluate(s, cfg(stage))


def mutate(fn, stage='E'):
    s = good(stage)
    fn(s)
    return s


CASES = {
    'C1': lambda s: s['nodes'].remove(RECOVERY),
    'C2': lambda s: s['lifecycle'].__setitem__(ARBITER, 'inactive'),
    'C3': lambda s: s['topics']['/helix/hold'].__setitem__('types', ['std_msgs/msg/Bool']),
    'C4': lambda s: s['topics']['/cmd_vel']['publishers'][0].__setitem__('reliability', BE),
    'C5': lambda s: s['topics']['/cmd_vel']['publishers'].append(ep('/rogue_teleop')),
    'C6': lambda s: s['topics']['/cmd_vel']['subscribers'].append(ep('/other_bridge')),
    'C7': lambda s: s.__setitem__('sink_mode', 'dry_run'),
    'C9': lambda s: s['state'].__setitem__('rate_hz', 3.0),
    'C10': lambda s: s['clock'].__setitem__('wall_now', 86400.0),
    'C11': lambda s: s.__setitem__('hold', {}),
    'C12': lambda s: s['status'].__setitem__('out_linear_x', 0.3),
    'C13': lambda s: s['topics']['/helix/hold']['publishers'].append(ep('/spoof')),
    'C11b': lambda s: s['hold'].__setitem__('hold', True),
}


@pytest.mark.parametrize('cid', sorted(CASES))
def test_each_condition_is_no_go(cid):
    s = mutate(CASES[cid])
    f = failed(s, cfg())
    assert cid in f and verdict(evaluate(s, cfg())) == 'NO-GO'


@pytest.mark.parametrize('rogue', [
    lambda s: s['nodes'].append('/twist_mux'),
    lambda s: s['topics'].__setitem__('/helix/cmd_vel', {
        'types': ['geometry_msgs/msg/Twist'], 'publishers': [ep(RECOVERY)],
        'subscribers': [ep('/twist_mux')]}),
    lambda s: s['topics']['/api/sport/request']['publishers'].append(ep('/unknown_ctrl')),
    lambda s: s['topics']['/api/sport/request']['publishers'].append(ep('/helix_other')),
    lambda s: s['topics']['/nav/cmd_vel'].__setitem__('subscribers', []) or
    s['topics']['/nav/cmd_vel']['publishers'].append(ep('/planner')),
], ids=['twist_mux', 'legacy_helix_cmd_vel', 'unknown_sport_pub', 'second_helix_sport_pub',
        'source_bypassing_arbiter'])
def test_competing_motion_authority_is_no_go(rogue):
    s = mutate(rogue)
    assert 'C8' in failed(s, cfg())


def test_dry_run_stage_forbids_helix_on_sport_topic():
    s = good('C')          # sink in dry_run, yet a HELIX node publishes Requests
    assert 'C8' in failed(s, cfg('C'))


def test_missing_baseline_is_no_go_after_stage_a():
    s = good('E')
    assert 'C8b' in failed(s, cfg('E', sport_baseline=None))


def test_robot_clock_skew_warns_but_does_not_block():
    s = mutate(lambda s: s['clock'].__setitem__('robot_skew_s', -3.0e7))
    checks = evaluate(s, cfg())
    assert [k.result for k in checks if k.id == 'C10b'] == ['WARN']
    assert verdict(checks) == 'GO'


def test_rehearsal_waives_only_the_baseline():
    s = good('E')
    assert verdict(evaluate(s, cfg('E', rehearsal=True, sport_baseline=None))) == 'GO'
    s['sink_mode'] = 'dry_run'
    assert 'C7' in failed(s, cfg('E', rehearsal=True, sport_baseline=None))


def test_input_not_mutated():
    s = good()
    before = copy.deepcopy(s)
    evaluate(s, cfg())
    assert s == before


def test_stale_state_only_warns_in_motors_off_stage_a():
    s = good('A')
    s['topics']['/api/sport/request']['publishers'] = [ep('/stock_a')]
    s['state'] = {'rate_hz': 0.0, 'last_age_s': float('inf')}
    checks = evaluate(s, cfg('A', sport_baseline=None))
    assert [k.result for k in checks if k.id == 'C9'] == ['WARN']
    assert verdict(checks) == 'GO'
