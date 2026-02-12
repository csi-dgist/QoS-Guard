#!/usr/bin/env python3
"""
Cyclone DDS: Code-only mode.

Eclipse Cyclone DDS does not support XML QoS profiles. When dds=cyclone,
QoS-Guard uses only code-based entity extraction (create_publisher,
create_subscription, QoS expressions in source code). XML files are ignored.
ROS 2 version is irrelevant for Cyclone—rules are applied to code-derived QoS.
"""
from typing import Any, Dict, List, Tuple


def parse_profile(xml: str) -> Dict[str, str | List[str]]:
    """Return empty dict—Cyclone has no XML QoS profiles."""
    return {}


def extract_all_entity_blocks(xml: str) -> List[Any]:
    """Return empty list—no XML entities for Cyclone (code-only)."""
    return []


def get_topic_profile_qos(
    full_xml: str, topic_name: str | None
) -> Dict[str, str | List[str]] | None:
    """Return None—no topic profiles from XML."""
    return None


def resolve_profile_for_block(
    full_xml: str, block: Any, dds: str = "cyclone"
) -> Tuple[Dict[str, str | List[str]], str, Dict[str, str], List[str]]:
    """Return empty QoS—all settings come from code."""
    return ({}, block.block_xml or "", {}, [])
