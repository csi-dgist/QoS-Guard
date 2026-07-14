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

"""Functional tests for package_scanner module."""

import pytest
from pathlib import Path

from qos_guard import package_scanner


def _qos_test_pkg_path() -> Path:
    """Return path to qos_test_pkg (sibling of qos_guard in workspace)."""
    return Path(__file__).resolve().parent.parent.parent / "qos_test_pkg"


@pytest.mark.functional
class TestFindAllXmlFiles:
    """Tests for find_all_xml_files."""

    def test_excludes_package_xml(self):
        """package.xml must not be included in results."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        files = package_scanner.find_all_xml_files(pkg)
        names = [f.name for f in files]
        assert "package.xml" not in names

    def test_finds_qos_profiles(self):
        """Should find at least 4 XML files (profiles + default_trap, etc.)."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        files = package_scanner.find_all_xml_files(pkg)
        assert len(files) >= 4


@pytest.mark.functional
class TestScanPackage:
    """Tests for scan_package."""

    def test_returns_entities(self):
        """scan_package should return a list of QosEntity."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        assert isinstance(entities, list)
        for e in entities:
            assert hasattr(e, "entity_type")
            assert hasattr(e, "topic_name")
            assert hasattr(e, "source_path")
            assert e.entity_type in ("pub", "sub")

    def test_has_code_only_entity_for_cmd_vel(self):
        """cmd_vel_pub is Code Only - no XML, from .cpp."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        code_pubs = [e for e in entities if e.entity_type == "pub" and not e.block_xml]
        cmd_vel_code = [e for e in code_pubs if e.topic_name and "cmd_vel" in e.topic_name]
        assert len(cmd_vel_code) >= 1
        assert cmd_vel_code[0].code_qos is not None

    def test_entities_have_node_name(self):
        """All entities should have node_name (executable or A,B,C fallback)."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        for e in entities:
            assert hasattr(e, "node_name")
            assert isinstance(e.node_name, str)
            assert len(e.node_name) > 0, f"node_name should not be empty: {e}"

    def test_has_entities_with_code_qos_override(self):
        """sensor_data, map, status should have code_qos merged from source."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        with_override = [e for e in entities if e.code_qos]
        assert len(with_override) >= 1

    def test_map_topic_both_xml_entities_get_code_qos(self):
        """1:N merge: if /map has 2 XML pubs, both should receive code_qos."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        map_pubs = [
            e for e in entities
            if e.entity_type == "pub"
            and e.topic_name
            and ("map" in e.topic_name or e.topic_name == "/map")
        ]
        assert len(map_pubs) >= 2, "map topic should have at least 2 pub entities"
        with_code = [e for e in map_pubs if e.code_qos]
        assert len(with_code) == len(map_pubs), (
            f"All {len(map_pubs)} map entities should have code_qos, got {len(with_code)}"
        )


@pytest.mark.functional
class TestBuildEntityPairs:
    """Tests for build_entity_pairs."""

    def test_produces_seven_pairs_for_qos_test_pkg(self):
        """build_entity_pairs must produce at least 7 pairs for qos_test_pkg."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        pairs = package_scanner.build_entity_pairs(entities)
        assert len(pairs) >= 7  # Pub-only/Sub-only synthetic pairs may add more

    def test_cmd_vel_pair_is_code_pub_with_xml_sub(self):
        """Topic /cmd_vel: Code pub + XML sub."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        pairs = package_scanner.build_entity_pairs(entities)
        cmd_vel_pairs = [
            (p, s) for p, s in pairs
            if p.topic_name and "cmd_vel" in p.topic_name
        ]
        assert len(cmd_vel_pairs) == 1
        pub, sub = cmd_vel_pairs[0]
        assert not pub.block_xml
        assert sub.block_xml
        assert "cmd_vel" in (pub.topic_name or "")

    def test_sensor_data_has_three_pairs(self):
        """Topic /sensor_data: 1 pub : 3 subscribers."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        pairs = package_scanner.build_entity_pairs(entities)
        sensor_pairs = [
            (p, s) for p, s in pairs
            if p.topic_name and "sensor_data" in p.topic_name
        ]
        assert len(sensor_pairs) == 3

    def test_map_has_two_pairs(self):
        """Topic /map: 2 publishers : 1 subscriber."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        pairs = package_scanner.build_entity_pairs(entities)
        map_pairs = [
            (p, s) for p, s in pairs
            if p.topic_name and (p.topic_name == "/map" or p.topic_name == "map")
        ]
        assert len(map_pairs) == 2

    def test_status_has_one_pair(self):
        """Topic /status: 1 pub : 1 sub."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = package_scanner.scan_package(pkg, dds="fast")
        pairs = package_scanner.build_entity_pairs(entities)
        status_pairs = [
            (p, s) for p, s in pairs
            if p.topic_name and "status" in p.topic_name
        ]
        assert len(status_pairs) == 1
