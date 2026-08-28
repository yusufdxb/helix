"""Repo-integrity check: the two AnomalyDetector implementations cannot drift.

HELIX ships the anomaly detector twice: `helix_core.anomaly_detector`
(Python, the default) and `helix_sensing_cpp::AnomalyDetectorNode` (C++,
behind `use_cpp_anomaly`). The C++ node is documented as a drop-in
behavioral replacement, which only holds if both sides declare the same
parameters with the same defaults, and if both read the same YAML.

Both invariants had already broken by the time these tests were written:

  * `emit_cooldown_s` existed only on the C++ side, so the cooldown line in
    the config was silently ignored whenever the Python node was launched
    (rclpy ignores overrides for undeclared parameters).
  * `helix_params.yaml` existed twice, in `helix_bringup/config/` and in
    `helix_sensing_cpp/config/`, and the copies had diverged in both the
    anomaly-detector block and in which nodes they configured at all.

These are source-text checks on purpose: they run in the no-ROS CI job, so
a drift is caught on every push rather than only when someone builds the
C++ package.
"""
from __future__ import annotations

import os
import re
import unittest

import yaml

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), os.pardir))
SRC = os.path.join(ROOT, "src")

PY_NODE = os.path.join(SRC, "helix_core", "helix_core", "anomaly_detector.py")
CPP_NODE = os.path.join(
    SRC, "helix_sensing_cpp", "src", "anomaly_detector_node.cpp"
)
CANONICAL_YAML = os.path.join(
    SRC, "helix_bringup", "config", "helix_params.yaml"
)

# self.declare_parameter("name", CONSTANT)
_PY_DECL = re.compile(
    r"self\.declare_parameter\(\s*\"([^\"]+)\"\s*,\s*([A-Za-z0-9_.+-]+)\s*,?\s*\)"
)
# DEFAULT_NAME: float = 3.0
_PY_CONST = re.compile(
    r"^([A-Z][A-Z0-9_]*)\s*:\s*(?:float|int|str|bool)\s*=\s*(\S+)\s*$",
    re.MULTILINE,
)
# declare_parameter<double>("name", 3.0);
_CPP_DECL = re.compile(
    r"declare_parameter<(\w+)>\(\s*\"([^\"]+)\"\s*,\s*([^)]+?)\s*\)\s*;"
)


def _num(text: str):
    """Normalize a numeric literal from either language to a comparable value."""
    text = text.strip().rstrip("fF")
    try:
        return float(text)
    except ValueError:
        return text


def _python_params() -> dict:
    with open(PY_NODE, encoding="utf-8") as fh:
        src = fh.read()
    consts = {m.group(1): m.group(2) for m in _PY_CONST.finditer(src)}
    out = {}
    for name, default in _PY_DECL.findall(src):
        # Defaults are declared as module constants; resolve through them so
        # the constant and the declaration cannot disagree either.
        resolved = consts.get(default, default)
        out[name] = _num(resolved)
    return out


def _cpp_params() -> dict:
    with open(CPP_NODE, encoding="utf-8") as fh:
        src = fh.read()
    return {
        name: _num(default) for _type, name, default in _CPP_DECL.findall(src)
    }


class TestParameterParity(unittest.TestCase):
    """The Python and C++ detectors must declare an identical parameter set."""

    def setUp(self):
        self.py = _python_params()
        self.cpp = _cpp_params()

    def test_parsers_found_parameters(self):
        # Guards against a refactor that silently defeats the regexes and
        # turns every assertion below into a vacuous pass.
        self.assertGreaterEqual(
            len(self.py), 4, f"parsed too few Python parameters: {self.py}"
        )
        self.assertGreaterEqual(
            len(self.cpp), 4, f"parsed too few C++ parameters: {self.cpp}"
        )

    def test_same_parameter_names(self):
        only_py = sorted(set(self.py) - set(self.cpp))
        only_cpp = sorted(set(self.cpp) - set(self.py))
        self.assertEqual(
            ([], []),
            (only_py, only_cpp),
            "AnomalyDetector parameter sets have drifted. "
            f"Python-only: {only_py}. C++-only: {only_cpp}. "
            "A parameter declared on one side only is silently ignored on "
            "the other, because both read the same YAML.",
        )

    def test_same_defaults(self):
        for name in sorted(set(self.py) & set(self.cpp)):
            self.assertEqual(
                self.py[name],
                self.cpp[name],
                f"default for '{name}' differs: "
                f"Python={self.py[name]} C++={self.cpp[name]}",
            )


class TestSingleParamsFile(unittest.TestCase):
    """Exactly one tracked helix_params.yaml may exist under src/."""

    def test_only_one_copy_under_src(self):
        found = []
        for dirpath, dirnames, filenames in os.walk(SRC):
            dirnames[:] = [
                d for d in dirnames if d not in ("build", "install", "log")
            ]
            if "helix_params.yaml" in filenames:
                found.append(
                    os.path.relpath(
                        os.path.join(dirpath, "helix_params.yaml"), ROOT
                    )
                )
        self.assertEqual(
            [os.path.relpath(CANONICAL_YAML, ROOT)],
            sorted(found),
            "helix_params.yaml must exist exactly once, in helix_bringup. "
            f"Found: {sorted(found)}. Two copies drift; that is why "
            "emit_cooldown_s and the heartbeat_monitor block disagreed.",
        )

    def test_launch_files_reference_the_canonical_package(self):
        launch = os.path.join(
            SRC,
            "helix_sensing_cpp",
            "launch",
            "anomaly_detector.launch.py",
        )
        with open(launch, encoding="utf-8") as fh:
            text = fh.read()
        self.assertIn(
            'FindPackageShare("helix_bringup"), "config", "helix_params.yaml"',
            text,
            "helix_sensing_cpp's launch file must resolve the shared config "
            "out of helix_bringup, not a package-local copy.",
        )


class TestYamlMatchesDeclarations(unittest.TestCase):
    """Every configured key must be declared by BOTH implementations."""

    def test_anomaly_detector_keys_are_declared(self):
        with open(CANONICAL_YAML, encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)
        params = cfg["helix_anomaly_detector"]["ros__parameters"]
        py = _python_params()
        cpp = _cpp_params()
        for key in params:
            self.assertIn(
                key,
                py,
                f"'{key}' is set in helix_params.yaml but not declared by "
                "the Python node, so rclpy will ignore it.",
            )
            self.assertIn(
                key,
                cpp,
                f"'{key}' is set in helix_params.yaml but not declared by "
                "the C++ node, so rclcpp will ignore it.",
            )


if __name__ == "__main__":
    unittest.main()
