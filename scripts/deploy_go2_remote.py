#!/usr/bin/env python3
"""Freeze a source bundle locally, or explicitly install one through LabLink.

No arguments prints a read-only plan. Only --remote-install contacts a device.
The remote operation builds in a new content-addressed home directory and
never starts nodes, changes services, or replaces an existing workspace.
"""
import argparse
import base64
import gzip
import hashlib
import io
import json
import re
import shlex
import subprocess
import tarfile
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[1]


def run(argv, **kwargs):
    return subprocess.run(argv, check=True, **kwargs)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def make_bundle(output, wheelhouse, unitree_source=None):
    # Freeze the working tree, including the explicitly scoped preparation
    # files. Never pick up untracked experiments, bags, credentials or logs.
    tracked = subprocess.check_output(["git", "-C", str(ROOT), "ls-files", "-z"])
    paths = {Path(p.decode()) for p in tracked.split(b"\0") if p}
    paths.update(p.relative_to(ROOT) for pattern in (
        "scripts/*deployment*", "scripts/deploy_go2*", "tests/test_deployment*",
        "src/helix_bringup/launch/helix_go2.launch.py",
        "src/helix_bringup/config/cyclonedds_offline.xml",
        "src/helix_bringup/test/test_go2_launch*.py", "docs/GO2_DEPLOYMENT_PREP.md",
    ) for p in ROOT.glob(pattern) if p.is_file())
    paths = {p for p in paths if p.parts[0] not in (".claude", ".github")
             and p.name not in ("CLAUDE.md", "AGENTS.md", "GEMINI.md")
             and p.suffix not in (".db3", ".mcap")}
    require(output.resolve() not in {(ROOT / p).resolve() for p in paths},
            "Bundle output must not overwrite a source file")
    output.parent.mkdir(parents=True, exist_ok=True)
    files = {}

    def add(archive, name, data, mode=0o644):
        entry = tarfile.TarInfo(name)
        entry.size, entry.mode, entry.mtime = len(data), mode, 0
        archive.addfile(entry, io.BytesIO(data))
        files[name] = hashlib.sha256(data).hexdigest()

    with output.open("wb") as output_file, gzip.GzipFile(
        filename="", mode="wb", fileobj=output_file, mtime=0
    ) as compressed, tarfile.open(fileobj=compressed, mode="w") as archive:
        for path in sorted(paths):
            source = ROOT / path
            if source.is_file():
                require(not source.is_symlink(), f"Refusing source symlink: {path}")
                add(archive, path.as_posix(), source.read_bytes(), source.stat().st_mode & 0o777)
        wheels = sorted(wheelhouse.glob("*.whl"))
        require(wheels, "No wheels: pip download -r scripts/requirements-deployment.txt --dest .deployment/wheels --only-binary=:all:")
        for path in wheels:
            require(path.name.endswith("-none-any.whl"), f"Target-independent wheels required: {path.name}")
            add(archive, f".deployment/wheels/{path.name}", path.read_bytes())
        vendor = None
        if unitree_source:
            repo = unitree_source.resolve()
            api = repo / "cyclonedds_ws/src/unitree/unitree_api"
            require((repo / "LICENSE").is_file() and (api / "package.xml").is_file(),
                    "Expected a unitree_ros2 checkout")
            vendor = {
                "upstream": "https://github.com/unitreerobotics/unitree_ros2",
                "revision": subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip(),
            }
            for path in sorted(api.rglob("*")):
                if path.is_file() and (path.suffix == ".msg" or path.name in ("CMakeLists.txt", "package.xml")):
                    require(not path.is_symlink(), f"Refusing Unitree source symlink: {path}")
                    add(archive, ".deployment/unitree-src/unitree_api/" + path.relative_to(api).as_posix(), path.read_bytes())
            add(archive, ".deployment/unitree-src/LICENSE", (repo / "LICENSE").read_bytes())
        metadata = {
            "schema": 1, "profile": "helix_go2.launch.py", "files": files,
            "source_revision": subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
            "working_tree_snapshot": True, "unitree_api": vendor,
        }
        add(archive, "DEPLOYMENT_BUNDLE.json", json.dumps(metadata, indent=2, sort_keys=True).encode())
    print(f"BUNDLE: {output}\nSHA256: {digest(output)}")


def inspect_bundle(path):
    with tarfile.open(path) as archive:
        members = archive.getmembers()
        names = set()
        for item in members:
            name = PurePosixPath(item.name)
            require(item.isfile() and not name.is_absolute() and ".." not in name.parts,
                    f"Unsafe bundle member: {item.name}")
            require(item.name not in names, f"Duplicate member {item.name}")
            names.add(item.name)
        meta = json.load(archive.extractfile("DEPLOYMENT_BUNDLE.json"))
        require(meta["schema"] == 1 and meta["profile"] == "helix_go2.launch.py",
                "Unsupported bundle schema or deployment profile")
        require(names == set(meta["files"]) | {"DEPLOYMENT_BUNDLE.json"},
                "Unexpected/missing bundle files")
        for name, expected in meta["files"].items():
            require(hashlib.sha256(archive.extractfile(name).read()).hexdigest() == expected,
                    f"Bundle checksum mismatch: {name}")
        for required in ("scripts/deploy_go2.sh", "scripts/check_deployment.py",
                         "src/helix_bringup/launch/helix_go2.launch.py"):
            require(required in names, f"Missing required deployment file: {required}")
    return meta


def remote_install(path, install_deps=False, device=None):
    require(isinstance(device, str) and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", device),
            "Specify a safe LabLink logical device name with --device")
    inspect_bundle(path)
    sha = digest(path)
    # Canonical logical target only. No addresses, ssh/scp fallback, trust edits
    # or force/unpinned options. A failed identity/status check stops here.
    doctor = subprocess.run(["lablink", "doctor", "--json", device], check=False,
                            capture_output=True, text=True)
    if doctor.returncode:
        diagnostic = "\n".join(part for part in (doctor.stdout, doctor.stderr) if part).strip()
        # Older LabLink CLIs lack doctor. Recognize only complete CLI errors
        # naming that missing subcommand; identity or transport failures and
        # mixed diagnostics never qualify. Status remains mandatory below.
        unsupported = any(re.fullmatch(pattern, diagnostic, flags=re.IGNORECASE) for pattern in (
            r"(?:Error: )?No such command ['\"]doctor['\"]\.?",
            r"(?:lablink: error: )?(?:unknown|unrecognized|unsupported) "
            r"(?:command|subcommand):? ['\"]?doctor['\"]?\.?",
            r"(?:usage:[^\n]*(?:\n[ \t]+[^\n]*)*\n)?"
            r"(?:lablink: error: )?argument (?:command|subcommand): "
            r"invalid choice: ['\"]doctor['\"] \(choose from [^\n]+\)",
        ))
        if not unsupported:
            doctor.check_returncode()
        print("LabLink doctor is unavailable; continuing with mandatory device status.")
    run(["lablink", "status", device, "--json"])

    def remote(command):
        run(["lablink", "exec", "--timeout", "1800", device, "bash -lc " + shlex.quote(command)])

    # Verify the target before creating even the transfer directory. Local
    # development builds still support both amd64 and arm64 Ubuntu hosts.
    remote("\n".join([
        "set -euo pipefail",
        "[[ $(uname -m) == aarch64 ]] || { echo 'Remote target requires aarch64' >&2; exit 1; }",
        "source /etc/os-release",
        "[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 22.04 ]] || "
        "{ echo 'Remote target requires Ubuntu 22.04' >&2; exit 1; }",
    ]))
    home = '"$HOME/helix-deploy"'
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    stage = f'{home}/bundles/{sha}.b64'
    remote(f'umask 077; mkdir -p {home}/bundles; : > {stage}')
    # LabLink's batch exec deliberately uses SSH -n, so do not assume stdin
    # forwarding. Each argument stays below Linux's per-argument size limit.
    for offset in range(0, len(encoded), 96000):
        remote(f"printf %s {shlex.quote(encoded[offset:offset + 96000])} >> {stage}")
    release = f'{home}/releases/{sha}'
    commands = [
        "set -euo pipefail", "umask 077",
        f"base64 -d {stage} > {home}/bundles/{sha}.tar.gz",
        f"printf '%s  %s\\n' {shlex.quote(sha)} {home}/bundles/{sha}.tar.gz | sha256sum -c -",
        f"mkdir -p {release}",
        f"if [[ -f {release}/.prepared.sha256 ]]; then grep -qx {sha} {release}/.prepared.sha256; echo 'Already prepared; no changes'; exit 0; fi",
        f"tar -xzf {home}/bundles/{sha}.tar.gz --no-same-owner -C {release}",
        f"cd {release}",
        "./scripts/deploy_go2.sh --dry-run",
    ]
    if install_deps:
        commands.append("./scripts/deploy_go2.sh --install-deps")
    commands += [
        "./scripts/deploy_go2.sh --check",
        "./scripts/deploy_go2.sh --build",
        f"printf '%s\\n' {sha} > .prepared.sha256",
        "echo 'Prepared. No nodes started. Source the release .deployment/env.bash manually.'",
    ]
    remote("\n".join(commands))
    print(f"Prepared release: ~/helix-deploy/releases/{sha}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--bundle", type=Path, help="Create a local source snapshot, including dirty tracked files")
    mode.add_argument("--remote-install", type=Path, help="Explicit remote build through LabLink, never launches nodes")
    mode.add_argument("--dry-run", type=Path, metavar="BUNDLE", help="Validate bundle and print remote plan, no connection")
    parser.add_argument("--wheelhouse", type=Path, default=ROOT / ".deployment/wheels")
    parser.add_argument("--unitree-source", type=Path, help="Include only unitree_api sources/license from this local unitree_ros2 checkout")
    parser.add_argument("--device", help="LabLink logical device name; required for --remote-install")
    parser.add_argument("--install-deps", action="store_true", help="Explicitly permit apt install during --remote-install")
    args = parser.parse_args()
    if args.install_deps and not args.remote_install:
        parser.error("--install-deps requires --remote-install")
    if args.remote_install and not args.device:
        parser.error("--remote-install requires an explicit --device")
    if args.bundle:
        make_bundle(args.bundle, args.wheelhouse, args.unitree_source)
    elif args.remote_install:
        remote_install(args.remote_install, args.install_deps, args.device)
    else:
        if args.dry_run:
            meta = inspect_bundle(args.dry_run)
            print(f"OK: {len(meta['files'])} files, SHA256={digest(args.dry_run)}")
        print("PLAN: explicit --remote-install --device <device> only: LabLink doctor -> status <device> -> read-only target check -> transfer -> checksum -> Humble check -> isolated build")
        print("PLAN: new release under ~/helix-deploy/releases/<bundle-sha256>; no service or node startup")


if __name__ == "__main__":
    main()
