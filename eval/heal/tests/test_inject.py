"""Regression tests for the composite injector's hits rule (python3 -m pytest eval/heal/tests).

c4_chain_late in the gen study: in the `none` arm nothing restarts rsp after the first deploy
TERMs it, so the second deploy (@18) finds no process. The old rule (`all` over every initial
part) turned that into hits = 0, recorded inject_subs = 0 and dropped every c4 block.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "humanoid"))
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
