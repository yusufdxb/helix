"""Repo-integrity check: the passive_adapter -> helix_adapter migration is complete.

The original `scripts/passive_adapter.py` was a monolithic Python script that
bridged GO2 standard topics into `/helix/metrics`. It has been replaced by the
packaged `helix_adapter` ROS 2 lifecycle nodes (topic_rate_monitor,
json_state_parser, pose_drift_monitor). The archived script remains in
`hardware_eval_20260406/scripts/` as evidence of what produced the Session 1
and Session 2 hardware results, but it is no longer the canonical adapter
path.

These tests guard against the migration regressing into a half-state:

  * `scripts/passive_adapter.py` must NOT be present on main (the half-state
    that triggered this migration was caused by docs/tests still pointing at
    that file after it was removed).
  * The packaged adapter must expose its three lifecycle entry points via
    setup.py — that is the contract the bringup launch and the README depend
    on.
  * `helix_bringup` must ship a launch file for the packaged adapter and that
    launch file must reference each adapter executable.

Pure-function helpers and lifecycle-node behavior are covered by the
`src/helix_adapter/test/` suite.
"""
from __future__ import annotations

import os
import re
import unittest

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), os.pardir))


class TestPassiveAdapterRemoved(unittest.TestCase):
    """The legacy monolithic adapter must not reappear in the canonical script dir."""

    def test_scripts_passive_adapter_absent(self):
        legacy = os.path.join(ROOT, "scripts", "passive_adapter.py")
        self.assertFalse(
            os.path.exists(legacy),
            "scripts/passive_adapter.py is the legacy monolithic adapter and "
            "should remain archived under hardware_eval_*/scripts/. Restore the "
            "packaged helix_adapter path instead of reintroducing this file.",
        )


class TestHelixAdapterEntryPoints(unittest.TestCase):
    """The packaged adapter must export the three canonical lifecycle nodes."""

    EXPECTED = (
        "helix_topic_rate_monitor",
        "helix_json_state_parser",
        "helix_pose_drift_monitor",
    )

    def setUp(self):
        with open(os.path.join(ROOT, "src", "helix_adapter", "setup.py")) as f:
            self.setup_text = f.read()

    def test_console_scripts_declared(self):
        for entry in self.EXPECTED:
            self.assertIn(
                entry, self.setup_text,
                f"helix_adapter/setup.py must declare console_script {entry!r}",
            )


class TestAdapterLaunchReferencesPackagedNodes(unittest.TestCase):
    """helix_bringup must launch the packaged helix_adapter executables."""

    def setUp(self):
        path = os.path.join(
            ROOT, "src", "helix_bringup", "launch", "helix_adapter.launch.py"
        )
        self.assertTrue(os.path.exists(path), f"missing launch file: {path}")
        with open(path) as f:
            self.launch_text = f.read()

    def test_launch_references_adapter_executables(self):
        for exec_name in TestHelixAdapterEntryPoints.EXPECTED:
            pattern = rf'executable\s*=\s*["\']{re.escape(exec_name)}["\']'
            self.assertRegex(
                self.launch_text, pattern,
                f"helix_adapter.launch.py must reference {exec_name!r} "
                "as a LifecycleNode executable",
            )

    def test_launch_uses_helix_adapter_package(self):
        self.assertIn('package="helix_adapter"', self.launch_text)


if __name__ == "__main__":
    unittest.main()
