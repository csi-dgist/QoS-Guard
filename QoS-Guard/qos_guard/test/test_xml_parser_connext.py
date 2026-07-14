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

"""Tests for RTI Connext XML parser."""

from pathlib import Path

import pytest

from qos_guard import xml_parser_connext as connext


def _test_xml_path(name: str) -> Path:
    """Return path to test XML file."""
    return Path(__file__).resolve().parent.parent / "test_xml" / name


class TestConnextParseProfile:
    """Tests for parse_profile."""

    def test_parses_reliability_durability_history(self):
        """Parse reliability, durability, history from RTI XML."""
        xml = """
        <datawriter_qos>
            <reliability><kind>RELIABLE_RELIABILITY_QOS</kind></reliability>
            <durability><kind>TRANSIENT_LOCAL_DURABILITY_QOS</kind></durability>
            <history><kind>KEEP_LAST_HISTORY_QOS</kind><depth>10</depth></history>
        </datawriter_qos>
        """
        q = connext.parse_profile(xml)
        assert q.get("reliability") == "RELIABLE_RELIABILITY_QOS"
        assert q.get("durability") == "TRANSIENT_LOCAL_DURABILITY_QOS"
        assert q.get("history") == "KEEP_LAST_HISTORY_QOS"
        assert q.get("history_depth") == "10"

    def test_partition_list_empty_when_no_partition(self):
        """partition_list returns [""] when no partition block."""
        xml = "<datawriter_qos><reliability><kind>RELIABLE</kind></reliability></datawriter_qos>"
        q = connext.parse_profile(xml)
        assert q.get("partition_list") == [""]


class TestConnextExtractEntityBlocks:
    """Tests for extract_all_entity_blocks."""

    def test_rti_dds_profiles_extracts_data_writer_reader_with_topic(self):
        """rti_dds_profiles format: data_writer/data_reader with topic_ref."""
        path = _test_xml_path("connext_humble2.xml")
        if not path.exists():
            pytest.skip("connext_humble2.xml not found")
        xml = path.read_text()
        blocks = connext.extract_all_entity_blocks(xml)
        assert len(blocks) >= 2
        tags = {b.tag for b in blocks}
        assert "data_writer" in tags
        assert "data_reader" in tags
        topics = [b.topic_name for b in blocks if b.topic_name]
        assert any("HelloWorld" in str(t) for t in topics)

    def test_rti_dds_qos_profiles_extracts_from_qos_profile(self):
        """rti_dds_qos_profiles format: qos_profile with datawriter/datareader_qos."""
        path = _test_xml_path("connext_humble1.xml")
        if not path.exists():
            pytest.skip("connext_humble1.xml not found")
        xml = path.read_text()
        blocks = connext.extract_all_entity_blocks(xml)
        assert len(blocks) >= 2
        # Shapes_Default_Profile has both datawriter and datareader
        profile_names = [b.profile_name for b in blocks]
        assert any("Shapes" in p for p in profile_names)


class TestConnextResolveProfile:
    """Tests for resolve_profile_for_block."""

    def test_resolves_base_name_from_qos_library(self):
        """base_name='qosLibrary::TransientDurability' resolves correctly."""
        path = _test_xml_path("connext_humble2.xml")
        if not path.exists():
            pytest.skip("connext_humble2.xml not found")
        xml = path.read_text()
        blocks = connext.extract_all_entity_blocks(xml)
        writer = next(b for b in blocks if b.tag == "data_writer")
        qos, entity_xml, src, logs = connext.resolve_profile_for_block(
            xml, writer, dds="connext"
        )
        assert qos.get("reliability") == "RELIABLE_RELIABILITY_QOS"
        assert qos.get("durability") == "TRANSIENT_LOCAL_DURABILITY_QOS"
