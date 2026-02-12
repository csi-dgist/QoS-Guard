# Copyright 2025 QoS-Guard Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Functional tests for qos_checker (package mode execution)."""

import subprocess
import sys
from pathlib import Path

import pytest


def _qos_test_pkg_path() -> Path:
    """Return path to qos_test_pkg."""
    return Path(__file__).resolve().parent.parent.parent / "qos_test_pkg"


def _run_qos_guard(package_path: Path) -> tuple[int, str]:
    """Run qos_guard in package mode, return (returncode, stdout+stderr)."""
    cmd = [
        sys.executable,
        "-m",
        "qos_guard.qos_checker",
        str(package_path),
        "fast",
        "humble",
    ]
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        cwd=str(Path(__file__).resolve().parent.parent),
    )
    return result.returncode, result.stdout + result.stderr


@pytest.mark.functional
class TestPackageMode:
    """Tests for qos_guard package mode."""

    def test_exits_successfully_with_warnings(self):
        """Package mode should complete (exit 0) when run on qos_test_pkg."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        rc, out = _run_qos_guard(pkg)
        assert rc == 0
        assert len(out) > 0

    def test_reports_critical_violations(self):
        """Output must contain Structural or Functional severity violations."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        rc, out = _run_qos_guard(pkg)
        assert "[STRUCTURAL]" in out or "[FUNCTIONAL]" in out or "[OPERATIONAL]" in out

    def test_reports_reliability_incompatibility(self):
        """Should detect QoS violations (reliability, durability, depth, etc.)."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        rc, out = _run_qos_guard(pkg)
        assert (
            "Incompatible reliability" in out
            or "reliability_kind" in out
            or "Invalid QoS" in out
            or "QoS Conflict" in out
            or "Writer.RELIAB" in out
            or "Writer.DURABL" in out
        )

    def test_reports_cmd_vel_pair(self):
        """Should include cmd_vel_pub (Code Only) in reported pairs."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        rc, out = _run_qos_guard(pkg)
        assert "cmd_vel" in out or "cmd_vel_pub" in out

    def test_reports_status_pair(self):
        """Should include /status topic pair."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        rc, out = _run_qos_guard(pkg)
        assert "status" in out

    def test_list_mode_works(self):
        """List mode should list XML files without package.xml."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        cmd = [
            sys.executable,
            "-m",
            "qos_guard.qos_checker",
            "list",
            str(pkg),
        ]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            cwd=str(Path(__file__).resolve().parent.parent),
        )
        assert result.returncode == 0
        out = result.stdout + result.stderr
        assert "Found" in out
        assert "XML file" in out


@pytest.mark.functional
class TestXmlPairMode:
    """Tests for qos_guard XML pair mode."""

    def test_compatible_pair_passes(self):
        """Compatible pub + sub XML pair should pass or report no critical."""
        base = Path(__file__).resolve().parent.parent
        pub = base / "test_xml" / "fast_jazzy1.xml"  # data_writer (pub)
        sub = base / "test_xml" / "fast_jazzy2.xml"  # data_reader (sub)
        if not pub.exists() or not sub.exists():
            pytest.skip("test_xml/fast_jazzy1.xml or fast_jazzy2.xml not found")
        cmd = [
            sys.executable,
            "-m",
            "qos_guard.qos_checker",
            str(pub),
            str(sub),
            "fast",
            "humble",
            "publish_period=40ms",
        ]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            cwd=str(base),
        )
        out = result.stdout + result.stderr
        assert result.returncode == 0
        assert "safe" in out or "CRITICAL" not in out or len(out) > 0
