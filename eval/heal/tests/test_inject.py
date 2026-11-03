"""Regression tests for the composite injector's hits rule (python3 -m pytest eval/heal/tests).

c4_chain_late in the gen study: in the `none` arm nothing restarts rsp after the first deploy
TERMs it, so the second deploy (@18) finds no process. The old rule (`all` over every initial
part) turned that into hits = 0, recorded inject_subs = 0 and dropped every c4 block.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "humanoid"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import inject  # noqa: E402
from inject import Injector  # noqa: E402

CHAIN = ("deploy:rsp:rsp.yaml:set:publish_frequency:120.0"
         "+deploy:rsp:rsp.yaml:set:publish_frequency:101.0@0.3")


def _run(tmp_path, monkeypatch, fault, pids):
    stack = tmp_path / "stack"
    stack.mkdir()
    (stack / "rsp.yaml").write_text("rsp:\n  ros__parameters:\n    publish_frequency: 50.0\n")
    logs = tmp_path / "logs"
    logs.mkdir()
    seq = iter(pids)
    monkeypatch.setattr(Injector, "_find", lambda self, node: next(seq, None))
    monkeypatch.setattr(Injector, "_signal", lambda self, pid, sig, what: (self._log(what, pid), True)[1])
    now = time.time()
    inj = Injector(fault, "0", {}, now, now + 1.0, logs, "t")
    inj.run()
    return inj, (stack / "rsp.yaml").read_text()


def test_delayed_deploy_without_live_process_keeps_injection(tmp_path, monkeypatch):
    # none arm: the first deploy TERMs pid 100, nothing restarts it, the delayed deploy finds none
    inj, text = _run(tmp_path, monkeypatch, CHAIN, [100, None])
    assert [x.hits for x in inj.subs] == [1, 0]
    assert inj.hits == 1
    assert inj.delayed_missed == ["deploy:rsp:rsp.yaml:set:publish_frequency:101.0"]
    assert "publish_frequency: 101.0" in text  # the second deploy was still written
    rec = inj.record()
    assert rec["hits"] == 1 and rec["parts"][1]["delay"] == 0.3


def test_both_deploys_reach_a_process(tmp_path, monkeypatch):
    # agent arms: the healer restarted rsp before +S, so both deploys TERM a process
    inj, _ = _run(tmp_path, monkeypatch, CHAIN, [100, 200])
    assert inj.hits == 2 and inj.delayed_missed == []


def test_undelayed_miss_still_voids_the_injection(tmp_path, monkeypatch):
    # the target is down at t_inject: the fault never reached it, the block must be dropped
    inj, _ = _run(tmp_path, monkeypatch, CHAIN, [None, 200])
    assert inj.hits == 0


def test_compose_hits_without_delay_attribute():
    class P:
        def __init__(self, hits, delay=0.0):
            self.hits, self.delay = hits, delay
    assert inject.compose_hits([P(1), P(2)]) == 3
    assert inject.compose_hits([P(1), P(0)]) == 0
    assert inject.compose_hits([P(1), P(0, 18.0)]) == 1


def test_fnv1a64_matches_healer_file_hash():
    # same vectors as Gen.AckParsesHexLinesOnly in test_agent.cpp
    assert inject.fnv1a64(b"") == 0xCBF29CE484222325
    assert inject.fnv1a64(b"a") == 0xAF63DC4C8601EC8C


def test_ack_appends_hash_of_file_on_disk_and_is_followup(tmp_path, monkeypatch):
    fault = "deploy:rsp:rsp.yaml:set:publish_frequency:120.0+ack:rsp:rsp.yaml@0.2"
    inj, text = _run(tmp_path, monkeypatch, fault, [100])
    ack = tmp_path / "stack/ack/rsp.ack"
    want = f"{inject.fnv1a64(text.encode()):016x}\n"
    assert "publish_frequency: 120.0" in text
    assert ack.read_text() == want
    assert inj.hits == 1 and inj.followup_hits == 1  # the ack never counts as an injection
    sub = Injector("ack:rsp:rsp.yaml", "0", {}, 0, 0, tmp_path / "logs", "t")
    sub.run()
    assert ack.read_text() == want * 2  # appends, never rewrites earlier acks


def test_genack_arm_is_gen_plus_episode_ack_dir():
    import run_episodes
    ep = dict(scenario="s", rep=0, dir="/x/ep", root="rsp", repair="restart")
    a = run_episodes.agent_params(ep, "genack@10")
    assert a[a.index("-p") + 1] == "agent_ranker:=gen"
    assert "agent_gen_ack_dir:=/x/ep/stack/ack" in a and "agent_lkg_hold_s:=10.0" in a
    assert not any("agent_gen_ack_dir" in x for x in run_episodes.agent_params(ep, "gen@10"))
