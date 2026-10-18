"""The supervised experiment option must never clear or release protection."""
import ast
from pathlib import Path
from types import SimpleNamespace


def test_keep_held_finishes_without_clear_or_release():
    source = (Path(__file__).resolve().parents[1] / 'scripts/hw/native_stop_test.py').read_text()
    tree = ast.parse(source)
    stage = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Stage')
    method = next(n for n in stage.body if isinstance(n, ast.FunctionDef) and n.name == 'finish_release')
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<finish_release>', 'exec'), namespace)
    calls = []

    def forbidden():
        raise AssertionError('the supervised experiment must not clear or release')

    io = SimpleNamespace(zero=lambda: calls.append('zero'), spin=lambda _: calls.append('observe'),
                         clear=forbidden, release=forbidden,
                         arb=SimpleNamespace(protective_latched=True))
    self = SimpleNamespace(io=io, a=SimpleNamespace(keep_held=True), stream_vx=.2,
                           check=lambda key, detail, ok: calls.append((key, ok)))
    namespace['finish_release'](self)
    assert self.stream_vx is None
    assert calls == ['zero', 'observe', ('kept_held', True)]
