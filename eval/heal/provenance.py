"""Build identity for every experiment output directory.

stamp(out) writes out/build.json: the repo HEAD, whether the working tree is dirty, the
commit and dirty flag the healer binary reports about itself (--build-info), the binary's
sha256, and whether the healer sources changed between the binary's commit and HEAD.
It refuses to run (SystemExit) when the binary is not traceable to a clean commit whose
healer sources match HEAD, unless allow_dirty is set; the override is recorded.
"""

import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HEALER = ROOT / "install/helix_healer/lib/helix_healer/helix_healer_node"
SOURCES = ["src/helix_healer", "src/helix_msgs"]


def _git(*args: str) -> str:
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout


def identity(healer: Path = HEALER) -> dict:
    head = _git("rev-parse", "HEAD").strip()
    dirty = [ln[3:] for ln in _git("status", "--porcelain", "--untracked-files=no").splitlines() if ln.strip()]
    out = subprocess.run([str(healer), "--build-info"], capture_output=True, text=True, timeout=10)
    try:
        binfo = json.loads(out.stdout)
    except json.JSONDecodeError:
        binfo = {"commit": "unknown", "dirty": True, "error": (out.stderr or out.stdout)[-300:]}
    src_changed = None
    if binfo.get("commit", "unknown") != "unknown":
        rc = subprocess.run(["git", "diff", "--quiet", binfo["commit"], "HEAD", "--", *SOURCES], cwd=ROOT).returncode
        src_changed = rc != 0
    return dict(
        head=head, tree_dirty=bool(dirty), dirty_files=dirty[:50],
        binary=dict(path=str(healer.relative_to(ROOT)), sha256=hashlib.sha256(healer.read_bytes()).hexdigest(),
                    mtime=time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(healer.stat().st_mtime)), **binfo),
        binary_sources_differ_from_head=src_changed,
        ros_distro=os.environ.get("ROS_DISTRO", ""), python=platform.python_version(), kernel=platform.release(),
        argv=sys.argv, started=time.strftime("%Y-%m-%dT%H:%M:%S"))


def problems(ident: dict) -> list:
    p = []
    if ident["tree_dirty"]:
        p.append(f"working tree dirty: {ident['dirty_files'][:5]}")
    if ident["binary"].get("dirty", True):
        p.append("healer binary was built from dirty sources")
    if ident["binary_sources_differ_from_head"] is not False:
        p.append(f"healer sources differ between binary commit {ident['binary'].get('commit')} and HEAD (rebuild)")
    return p


def stamp(out: Path, allow_dirty: bool = False, healer: Path = HEALER) -> dict:
    ident = identity(healer)
    ident["problems"] = problems(ident)
    ident["allow_dirty"] = allow_dirty
    if ident["problems"] and not allow_dirty:
        sys.exit("refusing to run an untraceable build:\n  " + "\n  ".join(ident["problems"]) +
                 "\ncommit and rebuild, or pass --allow-dirty (recorded in build.json)")
    out.mkdir(parents=True, exist_ok=True)
    (out / "build.json").write_text(json.dumps(ident, indent=1))
    return ident


if __name__ == "__main__":
    i = identity()
    print(json.dumps(dict(i, problems=problems(i)), indent=1))
