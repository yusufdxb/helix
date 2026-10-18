"""Topic remap for the dry stage variants: defaults, remap coverage, gating."""
import json

import pytest
from helix_arbiter.hw_stage import (
    PASS,
    REMAPPED_MARKER,
    gate,
    remap_refusal,
    session_refusal,
    topics_mode,
)
from helix_arbiter.preflight import (
    ARBITER,
    DEFAULT_TOPICS,
    DRY_PREFIX,
    REAL_MOTION_TOPICS,
    SINK,
    config_for,
    evaluate,
    resolve_topics,
    verdict,
)

from helix_arbiter import preflight

DRY = resolve_topics(prefix=DRY_PREFIX)
REAL = resolve_topics()


def _write(session, stage, **ev):
    d = session / f'stage_{stage}'
    d.mkdir(parents=True, exist_ok=True)
    ev.setdefault('verdict', PASS)
    ev.setdefault('git_sha', 'sha')
    ev.setdefault('config_hash', 'h')
    ev.setdefault('rehearsal', False)
    (d / 'evidence.json').write_text(json.dumps(ev))


# ---------------------------------------------------------------- defaults

def test_defaults_are_the_real_topics():
    assert REAL == {'mode': 'real', 'cmd': '/cmd_vel', 'nav': '/nav/cmd_vel',
                    'teleop': '/teleop/cmd_vel'}


def test_default_cli_is_unchanged():
    import argparse
    ap = argparse.ArgumentParser()
    preflight.add_topic_args(ap)
    assert preflight.topics_from_args(ap.parse_args([])) == REAL


def test_default_preflight_config_is_unchanged():
    assert config_for('E') == preflight.Config(stage='E')
    assert config_for('E', REAL) == preflight.Config(stage='E')


def test_passing_default_topics_explicitly_is_still_real():
    assert resolve_topics('/cmd_vel', '/nav/cmd_vel', '/teleop/cmd_vel') == REAL


def test_legacy_evidence_without_topics_key_is_real():
    assert topics_mode({'verdict': PASS}) == 'real'


# ------------------------------------------------------------------ remap

def test_prefix_moves_every_motion_topic():
    assert DRY == {'mode': 'remapped', 'cmd': '/helix_dry/cmd_vel',
                   'nav': '/helix_dry/nav/cmd_vel', 'teleop': '/helix_dry/teleop/cmd_vel'}


def test_cli_prefix_flag_and_explicit_topics():
    import argparse
    ap = argparse.ArgumentParser()
    preflight.add_topic_args(ap)
    assert preflight.topics_from_args(ap.parse_args(['--topic-prefix'])) == DRY
    t = preflight.topics_from_args(ap.parse_args(
        ['--cmd-topic', '/s/out', '--nav-topic', '/s/nav', '--teleop-topic', '/s/tel']))
    assert t == {'mode': 'remapped', 'cmd': '/s/out', 'nav': '/s/nav', 'teleop': '/s/tel'}


@pytest.mark.parametrize('kw', [
    {'cmd': '/s/out'},                                   # nav, teleop still real
    {'cmd': '/s/out', 'nav': '/s/nav'},                  # teleop still real
    {'cmd': '/api/sport/request', 'nav': '/s/n', 'teleop': '/s/t'},
    {'cmd': '/s/a', 'nav': '/s/a', 'teleop': '/s/t'},    # not distinct
    {'cmd': 's/a', 'nav': '/s/n', 'teleop': '/s/t'},     # relative
    {'cmd': '/s/a', 'prefix': '/x'},                     # both styles
    {'prefix': '/'},
])
def test_unsafe_or_partial_remap_rejected(kw):
    with pytest.raises(ValueError):
        resolve_topics(**kw)


def test_remapped_preflight_config_uses_remapped_topics():
    c = config_for('A', DRY)
    assert c.output_topic == DRY['cmd']
    assert set(c.source_topics) == {DRY['nav'], DRY['teleop']}
    assert c.remapped


def test_remap_only_for_dry_run_stages():
    assert preflight.REMAP_STAGES == ('A', 'C')
    for s in 'AC':
        assert remap_refusal(s, DRY) is None
    for s in 'BDEF':
        assert 'cannot run on remapped topics' in remap_refusal(s, DRY)
        assert remap_refusal(s, REAL) is None


# ---------------------------------------------------------------- gating

def test_remapped_evidence_never_unlocks_a_real_stage(tmp_path):
    _write(tmp_path, 'A', topics=DRY)
    why = gate('B', tmp_path, 'sha', 'h')
    assert why and 'remapped' in why
    for s in 'CDEF':        # further along the chain as well
        _write(tmp_path, s, topics=DRY)
    for s in 'BCDEF':
        prev = 'ABCDEF'['ABCDEF'.index(s) - 1]
        assert gate(s, tmp_path, 'sha', 'h') is not None, (s, prev)


def test_real_evidence_still_unlocks_real_stage(tmp_path):
    _write(tmp_path, 'A', topics=REAL)
    assert gate('B', tmp_path, 'sha', 'h') is None
    _write(tmp_path, 'D')             # legacy evidence, no topics key
    assert gate('E', tmp_path, 'sha', 'h') is None


def test_dry_chain_is_a_then_c(tmp_path):
    assert gate('A', tmp_path, 'sha', 'h', 'remapped') is None
    assert 'stage A evidence missing' in gate('C', tmp_path, 'sha', 'h', 'remapped')
    _write(tmp_path, 'A', topics=DRY)
    assert gate('C', tmp_path, 'sha', 'h', 'remapped') is None


def test_real_evidence_does_not_unlock_dry_stage(tmp_path):
    _write(tmp_path, 'A', topics=REAL)
    assert 'real' in gate('C', tmp_path, 'sha', 'h', 'remapped')


def test_dry_chain_keeps_sha_and_config_checks(tmp_path):
    _write(tmp_path, 'A', topics=DRY)
    assert 'SHA' in gate('C', tmp_path, 'other', 'h', 'remapped')
    assert 'config' in gate('C', tmp_path, 'sha', 'h2', 'remapped')


def test_session_dirs_never_mix_real_and_remapped(tmp_path):
    assert session_refusal(tmp_path, remapped=False) is None
    assert session_refusal(tmp_path, remapped=True) is None
    (tmp_path / REMAPPED_MARKER).touch()
    assert 'REMAPPED' in session_refusal(tmp_path, remapped=False)
    assert session_refusal(tmp_path, remapped=True) is None
    real = tmp_path / 'real'
    _write(real, 'A', topics=REAL)
    assert 'real-topic evidence' in session_refusal(real, remapped=True)


# ---------------------------------------------------- preflight on remap

def _dry_snapshot(real_extra=None):
    ep = [{'node': ARBITER, 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}]
    sk = [{'node': SINK, 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}]
    topics = {DRY['cmd']: {'types': ['geometry_msgs/msg/Twist'],
                           'publishers': ep, 'subscribers': sk}}
    topics.update(real_extra or {})
    return {'nodes': [ARBITER, SINK], 'topics': topics}


def test_c14_only_on_remapped_runs():
    assert 'C14' not in {c.id for c in evaluate(_dry_snapshot(), config_for('A'))}
    checks = {c.id: c for c in evaluate(_dry_snapshot(), config_for('A', DRY))}
    assert checks['C14'].result == 'PASS'
    assert checks['C5'].result == 'PASS' and checks['C6'].result == 'PASS'


@pytest.mark.parametrize('topic', REAL_MOTION_TOPICS)
@pytest.mark.parametrize('side', ['publishers', 'subscribers'])
def test_c14_fails_if_helix_touches_any_real_motion_topic(topic, side):
    ent = {'types': ['x'], 'publishers': [], 'subscribers': []}
    ent[side] = [{'node': '/helix_hw_stage', 'reliability': 'RELIABLE',
                  'durability': 'VOLATILE'}]
    checks = {c.id: c for c in evaluate(_dry_snapshot({topic: ent}), config_for('A', DRY))}
    assert checks['C14'].result == 'FAIL'
    assert verdict(list(checks.values())) == 'NO-GO'


def test_default_topics_constant_matches_config_defaults():
    c = preflight.Config()
    assert c.output_topic == DEFAULT_TOPICS['cmd']
    assert set(c.source_topics) == {DEFAULT_TOPICS['nav'], DEFAULT_TOPICS['teleop']}


# ------------------------------------------- runner's own ROS endpoints

@pytest.fixture
def ros():
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('helix_msgs.msg')
    rclpy.init()
    yield rclpy
    rclpy.shutdown()


BUILTIN = {'/parameter_events', '/rosout'}     # every rclpy node has these


def _endpoints(io):
    return ({p.topic_name for p in io.node.publishers} - BUILTIN,
            {s.topic_name for s in io.node.subscriptions} - BUILTIN)


def test_stage_io_default_endpoints_unchanged(ros):
    from helix_arbiter.hw_stage import StageIO
    io = StageIO(rehearsal=True)
    try:
        pubs, subs = _endpoints(io)
        assert pubs == {'/nav/cmd_vel', '/helix/faults'}
        assert {'/cmd_vel', '/nav/cmd_vel'} <= subs
    finally:
        io.node.destroy_node()


def test_stage_io_remap_applies_to_every_endpoint(ros):
    from helix_arbiter.hw_stage import StageIO
    io = StageIO(rehearsal=True, topics=DRY)
    try:
        pubs, subs = _endpoints(io)
        assert pubs == {DRY['nav'], '/helix/faults'}
        assert {DRY['cmd'], DRY['nav']} <= subs
        # no publisher or subscriber of the runner is on a real motion topic
        assert not (pubs | subs) & set(REAL_MOTION_TOPICS), (pubs, subs)
    finally:
        io.node.destroy_node()
