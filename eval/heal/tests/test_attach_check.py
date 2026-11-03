import importlib.util
from pathlib import Path

P = Path(__file__).resolve().parents[3] / "src/helix_healer/scripts/attach_check.py"
spec = importlib.util.spec_from_file_location("attach_check", P)
ac = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ac)


def fake_proc(root: Path, pid: int, ppid: int, start: int, argv: list[str], state: str = "S") -> None:
    d = root / str(pid)
    d.mkdir()
    (d / "cmdline").write_bytes(b"\0".join(a.encode() for a in argv) + b"\0")
    fields = [state, str(ppid)] + ["0"] * 17 + [str(start)] + ["0"] * 5
    (d / "stat").write_text(f"{pid} (x) " + " ".join(fields))


def test_patterns_and_adoption(tmp_path):
    text = "healer verify_s=5\nnode a act=0 attach=/exe_a  # c\nnode b attach=/exe_b\nnode c\nexec c x\n"
    assert ac.attach_patterns(text) == [("a", "/exe_a", False), ("b", "/exe_b", True)]
    fake_proc(tmp_path, 10, 1, 100, ["/usr/bin/python3", "/opt/lib/p/exe_a", "--ros-args"])
    fake_proc(tmp_path, 11, 1, 100, ["sh", "-c", "/opt/lib/p/exe_b"])         # wrapper
    fake_proc(tmp_path, 12, 11, 200, ["/opt/lib/p/exe_b"])                    # its child: adopted
    fake_proc(tmp_path, 13, 1, 300, ["/opt/lib/helix_healer/x", "/exe_a"])    # healer itself: ignored
    fake_proc(tmp_path, 14, 1, 400, ["/opt/lib/p/exe_a"], state="Z")          # zombie: ignored
    procs = ac.processes(str(tmp_path))
    n, best = ac.adopt("/exe_a", procs)
    assert (n, best["pid"]) == (1, 10)
    n, best = ac.adopt("/exe_b", procs)
    assert (n, best["pid"]) == (2, 12)
    assert ac.adopt("/exe_none", procs) == (0, None)
