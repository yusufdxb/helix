"""Offline deployment checks, including the read-only boundary of dry-run."""
from __future__ import annotations

import base64
import hashlib
import importlib.util
import io
import json
import os
import shlex
import shutil
import subprocess
import sys
import tarfile
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def source_tree(tmp_path):
    """Copy deployment inputs only, preserving the real launch/config files."""
    root = tmp_path / "helix"
    (root / "scripts").mkdir(parents=True)
    for name in ("deploy_go2.sh", "check_deployment.py"):
        shutil.copy2(ROOT / "scripts" / name, root / "scripts" / name)
    shutil.copy2(ROOT / "cyclonedds_loopback.xml", root)
    for manifest in (ROOT / "src").glob("*/package.xml"):
        package = root / "src" / manifest.parent.name
        package.mkdir(parents=True)
        shutil.copy2(manifest, package)
        for name in ("launch", "config"):
            if (manifest.parent / name).is_dir():
                shutil.copytree(manifest.parent / name, package / name,
                                ignore=shutil.ignore_patterns("__pycache__"))
    return root


def _snapshot(root):
    return {
        str(path.relative_to(root)): path.read_bytes() if path.is_file() else None
        for path in root.rglob("*")
    }


def _minimal_environment(tmp_path):
    env = os.environ.copy()
    for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
                "ROS_DISTRO", "ROS_VERSION", "VIRTUAL_ENV", "PYTHONPATH"):
        env.pop(key, None)
    # These tools must never execute in an offline dry-run, even if installed.
    blocked = tmp_path / "blocked-tools"
    blocked.mkdir()
    audit = tmp_path / "external-tool-called"
    for name in ("ssh", "scp", "rsync", "lablink", "ros2", "colcon", "sudo",
                 "apt-get", "pip", "pip3"):
        script = blocked / name
        script.write_text('#!/bin/sh\nprintf "%s\\n" "$0" >> "$DEPLOY_TEST_AUDIT"\nexit 97\n')
        script.chmod(0o755)
    # A sourced ROS installation must not be needed for source validation.
    for name in ("rclpy", "launch_ros", "ament_index_python"):
        (blocked / f"{name}.py").write_text(
            'raise RuntimeError("dry-run attempted a ROS import")\n'
        )
    env.update(PATH=f"{blocked}:/usr/bin:/bin", PYTHONPATH=str(blocked),
               PYTHONNOUSERSITE="1", DEPLOY_TEST_AUDIT=str(audit))
    return env, audit


@pytest.mark.parametrize("mode", [[], ["--dry-run"]])
def test_dry_run_requires_no_ros_or_remote_tools_and_writes_nothing(source_tree, tmp_path, mode):
    env, audit = _minimal_environment(tmp_path)
    state = tmp_path / "must-not-create-state"
    before = _snapshot(source_tree)
    arguments = [*mode, "--state-dir", str(state)] if mode else []
    result = subprocess.run(
        ["/bin/bash", str(source_tree / "scripts/deploy_go2.sh"), *arguments],
        env=env, text=True, capture_output=True, timeout=15,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "OK: source manifests" in result.stdout
    assert "sink dry_run, recovery disabled" in result.stdout
    assert not audit.exists(), audit.read_text() if audit.exists() else ""
    assert not state.exists()
    assert not (source_tree / ".deployment").exists()
    assert _snapshot(source_tree) == before


def test_invalid_deployment_mode_fails_without_creating_state(source_tree, tmp_path):
    env, audit = _minimal_environment(tmp_path)
    state = tmp_path / "must-not-create-state"
    result = subprocess.run(
        ["/bin/bash", str(source_tree / "scripts/deploy_go2.sh"), "--invalid-mode",
         "--state-dir", str(state)],
        env=env, text=True, capture_output=True, timeout=15,
    )
    assert result.returncode == 2
    assert "Unknown mode" in result.stderr
    assert not audit.exists()
    assert not state.exists()


@pytest.mark.parametrize("option", ["--state-dir", "--unitree-underlay", "--wheelhouse"])
@pytest.mark.parametrize("value", [[], ["--dry-run"]])
def test_missing_deployment_option_value_fails_before_any_action(source_tree, tmp_path, option, value):
    env, audit = _minimal_environment(tmp_path)
    before = _snapshot(source_tree)
    result = subprocess.run(
        ["/bin/bash", str(source_tree / "scripts/deploy_go2.sh"), "--dry-run", option, *value],
        env=env, text=True, capture_output=True, timeout=15,
    )
    assert result.returncode == 2
    assert "missing value" in result.stderr.lower()
    assert option in result.stderr
    assert "OK: source manifests" not in result.stdout
    assert not audit.exists()
    assert _snapshot(source_tree) == before


@pytest.fixture
def source_checker():
    spec = importlib.util.spec_from_file_location(
        "helix_deployment_checker", ROOT / "scripts/check_deployment.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.check_source


def test_source_checker_accepts_the_expected_packages(source_tree, source_checker):
    expected = source_checker.__globals__["EXPECTED_PACKAGES"]
    assert set(source_checker(source_tree)) == expected


@pytest.mark.parametrize("broken_input", [
    "manifest_xml", "duplicate_package", "missing_package", "launch_syntax",
    "missing_go2_profile", "dds_xml", "yaml_mapping", "detector_config", "missing_name",
])
def test_source_checker_rejects_bad_deployment_inputs(source_tree, source_checker, broken_input):
    manifest = source_tree / "src/helix_arbiter/package.xml"
    profile = source_tree / "src/helix_bringup/launch/helix_go2.launch.py"
    config = source_tree / "src/helix_bringup/config/helix_params.yaml"
    if broken_input == "manifest_xml":
        manifest.write_text("<package>")
    elif broken_input == "duplicate_package":
        manifest.write_text(manifest.read_text().replace("<name>helix_arbiter</name>",
                                                        "<name>helix_core</name>"))
    elif broken_input == "missing_package":
        manifest.unlink()
    elif broken_input == "missing_name":
        manifest.write_text(manifest.read_text().replace("<name>helix_arbiter</name>", ""))
    elif broken_input == "launch_syntax":
        profile.write_text("def broken(:\n")
    elif broken_input == "missing_go2_profile":
        profile.unlink()
    elif broken_input == "dds_xml":
        (source_tree / "cyclonedds_loopback.xml").write_text("<CycloneDDS>")
    elif broken_input == "yaml_mapping":
        config.write_text("- this is a sequence, not a parameter mapping\n")
    elif broken_input == "detector_config":
        config.write_text(config.read_text().replace("zscore_threshold: 4.0",
                                                     "zscore_threshold: 3.5"))
    with pytest.raises((ValueError, ET.ParseError, SyntaxError, KeyError, TypeError)):
        source_checker(source_tree)


def _dependencies(package, kinds):
    tree = ET.parse(ROOT / "src" / package / "package.xml").getroot()
    return {entry.text for kind in kinds for entry in tree.findall(kind)}


def test_python_manifests_use_resolvable_rosdep_dependency_names():
    for manifest in (ROOT / "src").glob("*/package.xml"):
        tree = ET.parse(manifest).getroot()
        dependencies = _dependencies(manifest.parent.name, (
            "depend", "build_depend", "buildtool_depend", "exec_depend", "test_depend",
        ))
        assert not dependencies.intersection({"pytest", "ament_python"}), manifest
        if tree.findtext("export/build_type") == "ament_python":
            assert "python3-setuptools" in dependencies, manifest


@pytest.mark.parametrize("package,required", [
    ("helix_bringup", {"helix_core", "helix_adapter", "helix_diagnosis", "helix_recovery",
                       "helix_explanation", "helix_arbiter", "launch_ros", "twist_mux",
                       "python3-yaml"}),
    ("helix_core", {"python3-yaml"}),
    ("helix_explanation", {"python3-requests"}),
    ("helix_adapter", {"helix_core", "lifecycle_msgs", "sensor_msgs", "nav_msgs"}),
])
def test_manifests_declare_required_deployment_runtime_dependencies(package, required):
    assert required <= _dependencies(package, ("depend", "exec_depend"))


@pytest.fixture
def remote_tool():
    spec = importlib.util.spec_from_file_location(
        "helix_remote_deployment", ROOT / "scripts/deploy_go2_remote.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_bundle(path, files=None, defect=None):
    files = files or {
        "scripts/deploy_go2.sh": b"#!/bin/bash\nexit 0\n",
        "scripts/check_deployment.py": b"# source checker\n",
        "src/helix_bringup/launch/helix_go2.launch.py": b"# fixed dry profile\n",
    }
    metadata = {"schema": 1, "profile": "helix_go2.launch.py", "files": {
        name: hashlib.sha256(data).hexdigest() for name, data in files.items()
    }}
    if defect == "checksum":
        metadata["files"]["scripts/deploy_go2.sh"] = "0" * 64
    elif defect == "missing":
        files = {name: data for name, data in files.items() if name != "scripts/deploy_go2.sh"}
    elif defect == "schema":
        metadata["schema"] = 999
    elif defect == "profile":
        metadata["profile"] = "armed.launch.py"
    elif defect == "required":
        files = {"README.md": b"not a deployable tree"}
        metadata["files"] = {"README.md": hashlib.sha256(files["README.md"]).hexdigest()}
    with tarfile.open(path, "w:gz") as archive:
        def add(name, data):
            entry = tarfile.TarInfo(name)
            entry.size = len(data)
            archive.addfile(entry, io.BytesIO(data))

        for name, data in files.items():
            add(name, data)
        if defect == "unexpected":
            add("unlisted.txt", b"unexpected")
        elif defect == "duplicate":
            add("scripts/deploy_go2.sh", b"overwritten")
        elif defect in ("traversal", "absolute"):
            add("../escape" if defect == "traversal" else "/escape", b"unsafe")
        elif defect == "symlink":
            entry = tarfile.TarInfo("external-link")
            entry.type = tarfile.SYMTYPE
            entry.linkname = "/tmp/escape"
            archive.addfile(entry)
        add("DEPLOYMENT_BUNDLE.json", json.dumps(metadata).encode())
    return metadata


@pytest.mark.parametrize("defect", [
    "checksum", "missing", "required", "unexpected", "duplicate", "traversal",
    "absolute", "symlink", "schema", "profile",
])
def test_bundle_rejects_corruption_and_unsafe_archive_members(remote_tool, tmp_path, defect):
    bundle = tmp_path / "corrupt.tar.gz"
    _write_bundle(bundle, defect=defect)
    with pytest.raises((ValueError, KeyError)):
        remote_tool.inspect_bundle(bundle)


@pytest.mark.parametrize("defect", ["checksum", "traversal"])
def test_archive_validation_cannot_be_disabled_by_python_optimization(tmp_path, defect):
    bundle = tmp_path / "invalid.tar.gz"
    _write_bundle(bundle, defect=defect)
    result = subprocess.run(
        [sys.executable, "-O", str(ROOT / "scripts/deploy_go2_remote.py"), "--dry-run", str(bundle)],
        text=True, capture_output=True, timeout=15,
    )
    assert result.returncode != 0
    assert "ValueError" in result.stderr
    assert "PLAN:" not in result.stdout


@pytest.mark.parametrize("arguments", [[], ["--dry-run"]])
def test_remote_plan_is_read_only_and_never_invokes_subprocesses(
        remote_tool, tmp_path, monkeypatch, capsys, arguments):
    bundle = tmp_path / "valid.tar.gz"
    metadata = _write_bundle(bundle)
    assert remote_tool.inspect_bundle(bundle) == metadata
    before = _snapshot(tmp_path)

    def forbidden(*args, **kwargs):
        pytest.fail("A read-only remote plan attempted a subprocess")

    monkeypatch.setattr(remote_tool, "subprocess", SimpleNamespace(run=forbidden, check_output=forbidden))
    monkeypatch.setattr(sys, "argv", ["deploy_go2_remote.py", *arguments,
                                    *([str(bundle)] if arguments else [])])
    remote_tool.main()
    assert "explicit --remote-install --device <device> only" in capsys.readouterr().out
    assert _snapshot(tmp_path) == before


def test_bundle_freezes_selected_working_tree_and_omits_private_or_untracked_inputs(
        remote_tool, source_tree, tmp_path, monkeypatch):
    (source_tree / "AGENTS.md").write_text("private agent instructions")
    (source_tree / ".claude").mkdir()
    (source_tree / ".claude/settings.json").write_text("private settings")
    (source_tree / "untracked-experiment.txt").write_text("do not transfer")
    (source_tree / "capture.mcap").write_bytes(b"recording excluded from deployment")
    (source_tree / "capture.db3").write_bytes(b"legacy recording excluded from deployment")
    launch_test = source_tree / "src/helix_bringup/test/test_go2_launch_integration.py"
    launch_test.parent.mkdir()
    launch_test.write_text("# uncommitted deployment integration check\n")
    script = source_tree / "scripts/deploy_go2.sh"
    script.write_text(script.read_text() + "\n# current uncommitted revision\n")
    tracked = ["scripts/deploy_go2.sh", "scripts/check_deployment.py",
               "src/helix_bringup/launch/helix_go2.launch.py", "AGENTS.md",
               ".claude/settings.json", "capture.mcap", "capture.db3"]

    def git(argv, **kwargs):
        assert argv[:3] == ["git", "-C", str(source_tree)]
        if "ls-files" in argv:
            return b"\0".join(name.encode() for name in tracked) + b"\0"
        assert argv[3:] == ["rev-parse", "HEAD"]
        return "a" * 40 + "\n"

    monkeypatch.setattr(remote_tool, "ROOT", source_tree)
    monkeypatch.setattr(remote_tool, "subprocess", SimpleNamespace(check_output=git))
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    (wheels / "test_dependency-1.0-py3-none-any.whl").write_bytes(b"wheel fixture")
    output = tmp_path / "bundle.tar.gz"
    remote_tool.make_bundle(output, wheels)
    metadata = remote_tool.inspect_bundle(output)
    assert metadata["files"]["scripts/deploy_go2.sh"] == hashlib.sha256(script.read_bytes()).hexdigest()
    assert not {"AGENTS.md", ".claude/settings.json", "untracked-experiment.txt",
                "capture.mcap", "capture.db3"}.intersection(metadata["files"])
    assert ".deployment/wheels/test_dependency-1.0-py3-none-any.whl" in metadata["files"]
    assert "src/helix_bringup/test/test_go2_launch_integration.py" in metadata["files"]
    repeated = tmp_path / "another-output-name.tar.gz"
    remote_tool.make_bundle(repeated, wheels)
    assert repeated.read_bytes() == output.read_bytes()


def _fake_lablink(remote_tool, monkeypatch, fail=None):
    calls = []

    def fake_run(argv, **kwargs):
        assert argv[0] == "lablink", f"Remote operation bypassed LabLink: {argv}"
        calls.append((argv, kwargs))
        preflight_failure = fail == "preflight" and argv[1] == "exec"
        code = 1 if argv[1] == fail or preflight_failure else 0
        if code and kwargs.get("check"):
            raise subprocess.CalledProcessError(code, argv, stderr="identity validation failed")
        return subprocess.CompletedProcess(argv, code)

    monkeypatch.setattr(remote_tool, "subprocess", SimpleNamespace(run=fake_run))
    return calls


@pytest.mark.parametrize("failed_step", ["doctor", "status", "preflight"])
def test_remote_identity_failure_prevents_all_transfer(remote_tool, tmp_path, monkeypatch, failed_step):
    bundle = tmp_path / "valid.tar.gz"
    _write_bundle(bundle)
    calls = _fake_lablink(remote_tool, monkeypatch, fail=failed_step)
    with pytest.raises(subprocess.CalledProcessError):
        remote_tool.remote_install(bundle, device="test-jetson")
    expected = {"doctor": ["doctor"], "status": ["doctor", "status"],
                "preflight": ["doctor", "status", "exec"]}
    assert [argv[1] for argv, _ in calls] == expected[failed_step]
    if failed_step == "preflight":
        assert "mkdir" not in calls[-1][0][-1]


@pytest.mark.parametrize("diagnostic,proceeds", [
    ("Error: No such command 'doctor'.", True),
    ("Unrecognized subcommand: doctor", True),
    ("usage: lablink [-h] {status,exec}\nlablink: error: argument command: "
     "invalid choice: 'doctor' (choose from 'status', 'exec')", True),
    ("Host-key mismatch for configured device", False),
    ("Error: No such command 'doctor'.\nHost-key mismatch for configured device", False),
    ("Network transport failed", False),
])
def test_only_missing_doctor_subcommand_allows_mandatory_status(
        remote_tool, tmp_path, monkeypatch, diagnostic, proceeds):
    bundle = tmp_path / "valid.tar.gz"
    _write_bundle(bundle)
    calls = []

    def fake_run(argv, **kwargs):
        assert argv[0] == "lablink"
        calls.append(argv)
        return subprocess.CompletedProcess(argv, 2 if argv[1] == "doctor" else 0,
                                           stdout="", stderr=diagnostic if argv[1] == "doctor" else "")

    monkeypatch.setattr(remote_tool, "subprocess", SimpleNamespace(run=fake_run))
    if proceeds:
        remote_tool.remote_install(bundle, device="test-jetson")
        assert calls[1] == ["lablink", "status", "test-jetson", "--json"]
        assert any(argv[1] == "exec" for argv in calls)
    else:
        with pytest.raises(subprocess.CalledProcessError):
            remote_tool.remote_install(bundle, device="test-jetson")
        assert len(calls) == 1


def test_invalid_bundle_prevents_even_lablink_diagnostics(remote_tool, tmp_path, monkeypatch):
    bundle = tmp_path / "invalid.tar.gz"
    _write_bundle(bundle, defect="checksum")
    calls = _fake_lablink(remote_tool, monkeypatch)
    with pytest.raises(ValueError):
        remote_tool.remote_install(bundle, device="test-jetson")
    assert not calls


def test_remote_cli_requires_an_explicit_device(remote_tool, tmp_path, monkeypatch):
    bundle = tmp_path / "valid.tar.gz"
    _write_bundle(bundle)
    calls = _fake_lablink(remote_tool, monkeypatch)
    monkeypatch.setattr(sys, "argv", ["deploy_go2_remote.py", "--remote-install", str(bundle)])
    with pytest.raises(SystemExit) as error:
        remote_tool.main()
    assert error.value.code == 2
    assert not calls


@pytest.mark.parametrize("device", [None, "", "-target", "host name", "user@host",
                                    "../target", "host;command", "host\nother"])
def test_remote_target_must_be_a_safe_logical_identifier(remote_tool, tmp_path, monkeypatch, device):
    bundle = tmp_path / "valid.tar.gz"
    _write_bundle(bundle)
    calls = _fake_lablink(remote_tool, monkeypatch)
    with pytest.raises(ValueError, match="logical device"):
        remote_tool.remote_install(bundle, device=device)
    assert not calls


@pytest.mark.parametrize("architecture,distribution,version,accepted", [
    ("aarch64", "ubuntu", "22.04", True),
    ("x86_64", "ubuntu", "22.04", False),
    ("aarch64", "ubuntu", "24.04", False),
    ("aarch64", "debian", "22.04", False),
])
def test_remote_target_preflight_checks_architecture_and_os_without_writes(
        remote_tool, tmp_path, monkeypatch, architecture, distribution, version, accepted):
    bundle = tmp_path / "valid.tar.gz"
    _write_bundle(bundle)
    calls = _fake_lablink(remote_tool, monkeypatch)
    remote_tool.remote_install(bundle, device="test-jetson")
    command = shlex.split(calls[2][0][-1])[2]
    assert "mkdir" not in command
    # Override only the two read-only OS queries and execute the actual
    # preflight shell, exercising rejection before any transfer command.
    queries = (
        f"uname() {{ printf '%s\\n' {shlex.quote(architecture)}; }}\n"
        f"source() {{ ID={shlex.quote(distribution)}; VERSION_ID={shlex.quote(version)}; }}\n"
    )
    before = _snapshot(tmp_path)
    result = subprocess.run(["/bin/bash", "-c", queries + command], text=True,
                            capture_output=True, timeout=15)
    assert (result.returncode == 0) is accepted
    assert _snapshot(tmp_path) == before


@pytest.mark.parametrize("install_deps", [False, True])
def test_remote_install_orders_verified_chunks_and_explicit_build_only(
        remote_tool, tmp_path, monkeypatch, install_deps):
    bundle = tmp_path / "valid.tar.gz"
    # Incompressible deterministic data forces more than one transfer chunk.
    payload = b"".join(hashlib.sha256(str(i).encode()).digest() for i in range(6000))
    files = {"scripts/deploy_go2.sh": b"#!/bin/bash\nexit 0\n",
             "scripts/check_deployment.py": b"# checker\n",
             "src/helix_bringup/launch/helix_go2.launch.py": payload}
    _write_bundle(bundle, files=files)
    calls = _fake_lablink(remote_tool, monkeypatch)
    remote_tool.remote_install(bundle, install_deps=install_deps, device="test-jetson")
    assert [argv for argv, _ in calls[:2]] == [
        ["lablink", "doctor", "--json", "test-jetson"],
        ["lablink", "status", "test-jetson", "--json"],
    ]
    shell_commands = []
    for argv, options in calls[2:]:
        assert options["check"] is True
        assert argv[:5] == ["lablink", "exec", "--timeout", "1800", "test-jetson"]
        shell = shlex.split(argv[5])
        assert shell[:2] == ["bash", "-lc"]
        shell_commands.append(shell[2])
    assert "uname -m" in shell_commands[0]
    assert "source /etc/os-release" in shell_commands[0]
    assert "mkdir" not in shell_commands[0]
    chunks = [shlex.split(command)[2] for command in shell_commands[2:-1]]
    assert len(chunks) > 1
    assert all(len(chunk) <= 96000 for chunk in chunks)
    assert base64.b64decode("".join(chunks)) == bundle.read_bytes()
    final = shell_commands[-1]
    ordered_steps = ["sha256sum -c -", ".prepared.sha256", "tar -xzf", "--dry-run",
                     "--check", "--build"]
    assert [final.index(step) for step in ordered_steps] == sorted(final.index(step) for step in ordered_steps)
    assert ("./scripts/deploy_go2.sh --install-deps" in final) is install_deps
    for forbidden in ("ssh ", "scp ", "rsync ", "ros2 launch", "ros2 run", "systemctl ", "service "):
        assert forbidden not in "\n".join(shell_commands)


@pytest.mark.parametrize("matching_marker", [False, True])
def test_remote_release_marker_exits_before_extraction_or_build(
        remote_tool, tmp_path, monkeypatch, matching_marker):
    bundle = tmp_path / "valid.tar.gz"
    _write_bundle(bundle)
    calls = _fake_lablink(remote_tool, monkeypatch)
    remote_tool.remote_install(bundle, device="test-jetson")
    command = shlex.split(calls[-1][0][-1])[2]
    sha = remote_tool.digest(bundle)
    home = tmp_path / "isolated-home"
    release = home / "helix-deploy" / "releases" / sha
    release.mkdir(parents=True)
    marker = release / ".prepared.sha256"
    marker.write_text((sha if matching_marker else "wrong digest") + "\n")
    bundles = home / "helix-deploy" / "bundles"
    bundles.mkdir()
    (bundles / f"{sha}.b64").write_bytes(base64.b64encode(bundle.read_bytes()))
    # Executing only this captured shell locally exercises the marker guard.
    # If tar were reached, this stub fails and records the regression.
    tools = tmp_path / "shell-tools"
    tools.mkdir()
    audit = tmp_path / "tar-called"
    tar = tools / "tar"
    tar.write_text('#!/bin/sh\nprintf called > "$DEPLOY_TEST_AUDIT"\nexit 97\n')
    tar.chmod(0o755)
    env = {**os.environ, "HOME": str(home), "PATH": f"{tools}:/usr/bin:/bin",
           "DEPLOY_TEST_AUDIT": str(audit)}
    result = subprocess.run(["/bin/bash", "-c", command], env=env, text=True,
                            capture_output=True, timeout=15)
    assert (result.returncode == 0) is matching_marker
    assert ("Already prepared; no changes" in result.stdout) is matching_marker
    assert not audit.exists()
    assert marker.read_text() == (sha if matching_marker else "wrong digest") + "\n"
