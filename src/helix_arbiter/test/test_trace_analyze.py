"""Pure tests of the trace analyzer (chain correlation and hold-window counting)."""
from helix_arbiter.trace import analyze


def _chain_events(release_output_before_status):
    E = []

    def ev(kind, t, **kw):
        E.append(dict(kind=kind, t=t, **kw))
    ev('fault', 0.000, node_name='m', src_stamp=100.000)
    ev('hint', 0.001, fault_id='m', suggested_action='STOP_AND_HOLD')
    ev('action', 0.002, fault_id='m', action='STOP_AND_HOLD', status='ACCEPTED',
       src_stamp=100.002)
    ev('hold', 0.003, hold=True, fault_id='m', src_stamp=100.003)
    for k in range(5):                       # 5 held ticks, output then status
        t = 0.004 + 0.02 * k
        ev('output', t, zero=True, vx=0.0)
        ev('status', t + 0.0002, reason='HELIX_HOLD', hold_fault_id='m',
           src_stamp=100.004 + 0.02 * k)
    t = 0.004 + 0.02 * 5                     # first released tick, moving again
    if release_output_before_status:
        ev('output', t, zero=False, vx=0.2)
        ev('status', t + 0.0002, reason='SOURCE', hold_fault_id='', src_stamp=100.2)
    else:
        ev('status', t, reason='SOURCE', hold_fault_id='', src_stamp=100.2)
        ev('output', t + 0.0002, zero=False, vx=0.2)
    return E


def test_first_released_output_is_not_counted_as_during_hold():
    for order in (True, False):
        res = analyze(_chain_events(order))
        assert res['nonzero_outputs_during_hold'] == 0


def test_real_leak_during_hold_is_counted():
    E = _chain_events(True)
    E.append(dict(kind='output', t=0.05, zero=False, vx=0.3))   # between held ticks
    assert analyze(E)['nonzero_outputs_during_hold'] == 1


def test_chain_complete_and_source_latency():
    c = analyze(_chain_events(True))['chains'][0]
    assert c['complete_to_output']
    assert abs(c['fault_to_output_src_ms'] - 4.0) < 1e-6
    assert abs(c['src_deltas_ms']['fault->action_ms'] - 2.0) < 1e-6
