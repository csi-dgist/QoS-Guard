#!/usr/bin/env python3
"""
XML parsing Facade.

Call appropriate parser according to DDS vendor.
- fast: xml_parser_fastdds
- cyclone: xml_parser_cyclone
- connext: xml_parser_connext
"""
from typing import Dict, List, Tuple

from . import xml_parser_connext
from . import xml_parser_cyclone
from . import xml_parser_fastdds

_PARSERS = {
    "fast": xml_parser_fastdds,
    "cyclone": xml_parser_cyclone,
    "connext": xml_parser_connext,
}

# Fast DDS EntityBlock type re-export
EntityBlock = xml_parser_fastdds.EntityBlock


def parse_profile(xml: str, dds: str = "fast") -> Dict[str, str | list[str]]:
    """Parse QoS profile from XML string and return dict."""
    parser = _PARSERS.get(dds)
    if parser is None:
        raise ValueError(f"Unknown DDS vendor: {dds}. Choose from {list(_PARSERS)}")
    return parser.parse_profile(xml)


def resolve_profile(
    xml: str, role: str, dds: str = "fast"
) -> Tuple[Dict[str, str | list[str]], str, Dict[str, str], List[str]]:
    """
    Merge topic and entity QoS by 5-level priority rules.

    Returns
    -------
    tuple
        (qos_dict, entity_xml, source_info, log_msgs)

    """
    parser = _PARSERS.get(dds)
    if parser is None:
        raise ValueError(f"Unknown DDS vendor: {dds}. Choose from {list(_PARSERS)}")
    if not hasattr(parser, "resolve_profile"):
        qos = parser.parse_profile(xml)
        return qos, xml, {}, []
    return parser.resolve_profile(xml, role, dds)


def extract_all_entity_blocks(xml: str, dds: str = "fast") -> List["EntityBlock"]:
    """Extract all publisher, subscriber, data_writer, data_reader, topic blocks."""
    parser = _PARSERS.get(dds)
    if parser is None:
        raise ValueError(f"Unknown DDS vendor: {dds}. Choose from {list(_PARSERS)}")
    if not hasattr(parser, "extract_all_entity_blocks"):
        return []
    return parser.extract_all_entity_blocks(xml)


def get_topic_profile_qos(full_xml: str, topic_name: str | None, dds: str = "fast") -> "Dict[str, str | list[str]] | None":
    """Return QoS from XML topic profile matching topic_name (for code-only entities)."""
    parser = _PARSERS.get(dds)
    if parser is None or not hasattr(parser, "get_topic_profile_qos"):
        return None
    return parser.get_topic_profile_qos(full_xml, topic_name)


def resolve_profile_for_block(
    full_xml: str, block: "EntityBlock", dds: str = "fast"
) -> Tuple[Dict[str, str | list[str]], str, Dict[str, str], List[str]]:
    """
    Merge QoS for single entity block by 5-level priority rules.

    Returns
    -------
    tuple
        (qos_dict, entity_xml, source_info, log_msgs)

    """
    parser = _PARSERS.get(dds)
    if parser is None:
        raise ValueError(f"Unknown DDS vendor: {dds}. Choose from {list(_PARSERS)}")
    if not hasattr(parser, "resolve_profile_for_block"):
        qos = parser.parse_profile(block.block_xml)
        return qos, block.block_xml, {}, []
    return parser.resolve_profile_for_block(full_xml, block, dds)
