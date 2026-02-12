#!/usr/bin/env python3
"""
RTI Connext DDS QoS profile XML parser.

Supported formats:
- rti_dds_qos_profiles.xsd: <qos_library>/<qos_profile> with datawriter_qos, datareader_qos
- rti_dds_profiles.xsd: <domain_participant_library> with data_writer/data_reader topic_ref
"""
import re
from typing import Dict, List, Tuple

from . import xml_parser_fastdds
from .xml_parser_fastdds import EntityBlock

# Re-export for rules compatibility (RTI XML structure similar to DDS standard)
deadline_enabled = xml_parser_fastdds.deadline_enabled
deadline_period_ns = xml_parser_fastdds.deadline_period_ns
lease_duration_ns = xml_parser_fastdds.lease_duration_ns
parse_duration_field = xml_parser_fastdds.parse_duration_field
is_inf = xml_parser_fastdds.is_inf
DEADLINE_RE = xml_parser_fastdds.DEADLINE_RE
LEASE_RE = xml_parser_fastdds.LEASE_RE
ANNOUNCE_RE = xml_parser_fastdds.ANNOUNCE_RE
LIFESPAN_RE = xml_parser_fastdds.LIFESPAN_RE
INF_SET = xml_parser_fastdds.INF_SET

# RTI uses <partition><name>...</name></partition>
PART_ALL_RE = re.compile(r"<\s*partition\s*>.*?</\s*partition\s*>", re.I | re.S)
NAME_RE = re.compile(r"<\s*name\s*>([^<]+)</name\s*>", re.I | re.S)


def partition_list(xml: str) -> list[str]:
    """Extract partition name list from RTI XML."""
    blk = PART_ALL_RE.search(xml)
    if not blk:
        return [""]
    names = NAME_RE.findall(blk.group(0))
    return [n.strip() for n in names] if names else [""]


# RTI Connext tag names (history, resource_limits, writer_data_lifecycle)
# Output keys match rules expectation
RTI_TAG_PATTERNS = {
    "reliability": re.compile(
        r"<\s*reliability\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*reliability\s*>",
        re.I | re.S,
    ),
    "history": re.compile(
        r"<\s*history\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*history\s*>",
        re.I | re.S,
    ),
    "history_depth": re.compile(
        r"<\s*history\s*>.*?<\s*depth\s*>(\d+)</depth\s*>", re.I | re.S
    ),
    "durability": re.compile(
        r"<\s*durability\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*durability\s*>",
        re.I | re.S,
    ),
    "ownership": re.compile(
        r"<\s*ownership\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*ownership\s*>",
        re.I | re.S,
    ),
    "dest_order": re.compile(
        r"<\s*destination_order\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?"
        r"</\s*destination_order\s*>",
        re.I | re.S,
    ),
    "max_samples": re.compile(
        r"<\s*resource_limits\s*>.*?<\s*max_samples\s*>(\d+)</max_samples\s*>",
        re.I | re.S,
    ),
    "max_instances": re.compile(
        r"<\s*resource_limits\s*>.*?<\s*max_instances\s*>(\d+)</max_instances\s*>",
        re.I | re.S,
    ),
    "max_samples_per_instance": re.compile(
        r"<\s*resource_limits\s*>.*?<\s*max_samples_per_instance\s*>(\d+)"
        r"</max_samples_per_instance\s*>",
        re.I | re.S,
    ),
    "autodispose": re.compile(
        r"<\s*writer_data_lifecycle\s*>.*?"
        r"<\s*autodispose_unregistered_instances\s*>\s*(\w+)\s*"
        r"</autodispose_unregistered_instances\s*>.*?"
        r"</\s*writer_data_lifecycle\s*>",
        re.I | re.S,
    ),
    "liveliness": re.compile(
        r"<\s*liveliness\s*>.*?<\s*kind\s*>(.*?)</kind\s*>.*?</\s*liveliness\s*>",
        re.I | re.S,
    ),
}


def parse_profile(xml: str) -> Dict[str, str | list[str]]:
    """Parse QoS from RTI Connext XML (datawriter_qos/datareader_qos block)."""
    out: Dict[str, str | list[str]] = {}
    for k, pat in RTI_TAG_PATTERNS.items():
        m = pat.search(xml)
        out[k] = (m.group(1).strip().upper() if m else "")
    out["partition_list"] = partition_list(xml)
    return out


def _find_qos_profile_blocks(xml: str) -> List[Tuple[str, str, bool, str | None]]:
    """Extract <qos_profile name="X" is_default_qos="true" base_name="Y">...</qos_profile>."""
    results: List[Tuple[str, str, bool, str | None]] = []
    pattern = r"<\s*qos_profile\s+([^>]*)>"
    for m in re.finditer(pattern, xml, re.I):
        attrs = m.group(1)
        name_m = re.search(r'name\s*=\s*["\']([^"\']+)["\']', attrs, re.I)
        default_m = re.search(
            r'is_default_qos\s*=\s*["\']?(true|false)["\']?', attrs, re.I
        )
        base_m = re.search(r'base_name\s*=\s*["\']([^"\']+)["\']', attrs, re.I)
        profile_name = name_m.group(1).strip() if name_m else ""
        is_default = default_m and default_m.group(1).lower() == "true"
        base_name = base_m.group(1).strip() if base_m else None

        start = m.end()
        depth = 1
        i = start
        tag = "qos_profile"
        while i < len(xml) and depth > 0:
            open_m = re.search(rf"<\s*{tag}\s", xml[i:], re.I)
            close_m = re.search(rf"<\s*/\s*{tag}\s*>", xml[i:], re.I)
            open_pos = open_m.start() + i if open_m else len(xml)
            close_pos = close_m.start() + i if close_m else len(xml)
            if close_m and (not open_m or close_pos < open_pos):
                depth -= 1
                if depth == 0:
                    block = xml[start:i + close_m.end()]
                    results.append((profile_name, block, is_default, base_name))
                    break
                i = close_pos + len(close_m.group(0))
            elif open_m:
                depth += 1
                i = open_pos + len(open_m.group(0))
            else:
                break
    return results


def _extract_tag_block(xml: str, tag: str) -> str | None:
    """Extract first <tag>...</tag> block content."""
    pat = rf"<\s*{tag}\s*>([\s\S]*?)<\s*/\s*{tag}\s*>"
    m = re.search(pat, xml, re.I)
    return m.group(1) if m else None


def _extract_data_writer_reader_blocks(xml: str) -> List[Tuple[str, str, str | None]]:
    """
    Extract <data_writer topic_ref="X"> and <data_reader topic_ref="X"> from
    domain_participant_library.
    Returns (tag, block_xml, topic_ref).
    """
    results: List[Tuple[str, str, str | None]] = []
    for tag in ("data_writer", "data_reader"):
        pattern = rf"<\s*{tag}\s+([^>]*)>"
        for m in re.finditer(pattern, xml, re.I):
            attrs = m.group(1)
            topic_m = re.search(r'topic_ref\s*=\s*["\']([^"\']+)["\']', attrs, re.I)
            topic_ref = topic_m.group(1).strip() if topic_m else None

            start = m.end()
            depth = 1
            i = start
            while i < len(xml) and depth > 0:
                open_m = re.search(rf"<\s*{tag}\s", xml[i:], re.I)
                close_m = re.search(rf"<\s*/\s*{tag}\s*>", xml[i:], re.I)
                open_pos = open_m.start() + i if open_m else len(xml)
                close_pos = close_m.start() + i if close_m else len(xml)
                if close_m and (not open_m or close_pos < open_pos):
                    depth -= 1
                    if depth == 0:
                        block = xml[start:i + close_m.end()]
                        results.append((tag, block, topic_ref))
                        break
                    i = close_pos + len(close_m.group(0))
                elif open_m:
                    depth += 1
                    i = open_pos + len(open_m.group(0))
                else:
                    break
    return results


def _resolve_topic_ref(xml: str, topic_ref: str) -> str:
    """Resolve topic_ref (e.g. HelloWorldTopic) to full topic name. Add / prefix."""
    if not topic_ref:
        return ""
    t = topic_ref.strip()
    if t.startswith("/"):
        return t
    return f"/{t}"


def _find_qos_libraries(xml: str) -> Dict[str, str]:
    """Extract qos_library name -> full library block XML."""
    result: Dict[str, str] = {}
    pattern = r"<\s*qos_library\s+name\s*=\s*[\"']([^\"']+)[\"']\s*([^>]*)>"
    for m in re.finditer(pattern, xml, re.I):
        lib_name = m.group(1).strip()
        start = m.end()
        depth = 1
        i = start
        tag = "qos_library"
        while i < len(xml) and depth > 0:
            open_m = re.search(rf"<\s*{tag}\s", xml[i:], re.I)
            close_m = re.search(rf"<\s*/\s*{tag}\s*>", xml[i:], re.I)
            open_pos = open_m.start() + i if open_m else len(xml)
            close_pos = close_m.start() + i if close_m else len(xml)
            if close_m and (not open_m or close_pos < open_pos):
                depth -= 1
                if depth == 0:
                    result[lib_name] = xml[start:i + close_m.end()]
                    break
                i = close_pos + len(close_m.group(0))
            elif open_m:
                depth += 1
                i = open_pos + len(open_m.group(0))
            else:
                break
    return result


def _resolve_qos_ref(xml: str, base_name: str) -> str | None:
    """Resolve base_name (Library::Profile) to full qos_profile block XML."""
    if not base_name:
        return None
    parts = base_name.split("::")
    profile_name = parts[-1].strip()
    lib_name = parts[0].strip() if len(parts) > 1 else None

    if lib_name:
        libs = _find_qos_libraries(xml)
        search_xml = libs.get(lib_name, xml)
    else:
        search_xml = xml

    profiles = _find_qos_profile_blocks(search_xml)
    for pname, block, _, _ in profiles:
        if pname == profile_name:
            return block
    return None


def extract_all_entity_blocks(xml: str) -> List[EntityBlock]:
    """Extract all publisher, subscriber, data_writer, data_reader from RTI XML."""
    result: List[EntityBlock] = []

    # Format 1: rti_dds_profiles - domain_participant_library with data_writer/data_reader
    # Prefer this when present (explicit topic association)
    has_participant_lib = "domain_participant_library" in xml or "domain_participant" in xml
    if has_participant_lib:
        dw_dr = _extract_data_writer_reader_blocks(xml)
        for tag, block_xml, topic_ref in dw_dr:
            topic_name = _resolve_topic_ref(xml, topic_ref) if topic_ref else None
            # datawriter_qos/datareader_qos can be inline or reference base_name="Lib::Profile"
            base_m = re.search(
                r'(?:datawriter_qos|datareader_qos)[^>]*base_name\s*=\s*["\']([^"\']+)["\']',
                block_xml,
                re.I,
            )
            base_name = base_m.group(1).strip() if base_m else None
            name_m = re.search(
                r'(?:datawriter_qos|datareader_qos)[^>]*name\s*=\s*["\']([^"\']+)["\']',
                block_xml,
                re.I,
            )
            profile_name = (name_m.group(1).strip() if name_m else None) or base_name or tag
            result.append(
                EntityBlock(
                    tag=tag,
                    profile_name=profile_name,
                    topic_name=topic_name,
                    block_xml=block_xml,
                    is_default=False,
                    base_profile_name=base_name,
                )
            )
        return result  # Use only participant format when present

    # Format 2: rti_dds_qos_profiles - qos_profile with datawriter_qos, datareader_qos
    profiles = _find_qos_profile_blocks(xml)
    for profile_name, block, is_default, base_name in profiles:
        dw_block = _extract_tag_block(block, "datawriter_qos")
        dr_block = _extract_tag_block(block, "datareader_qos")
        if dw_block:
            result.append(
                EntityBlock(
                    tag="data_writer",
                    profile_name=profile_name,
                    topic_name=None,
                    block_xml=dw_block,
                    is_default=is_default,
                    base_profile_name=base_name,
                )
            )
        if dr_block:
            result.append(
                EntityBlock(
                    tag="data_reader",
                    profile_name=profile_name,
                    topic_name=None,
                    block_xml=dr_block,
                    is_default=is_default,
                    base_profile_name=base_name,
                )
            )

    return result


def _resolve_base_profile(
    full_xml: str, base_name: str | None, tag: str
) -> Dict[str, str | list[str]]:
    """Resolve base_name to QoS dict. Handles Library::Profile format."""
    if not base_name:
        return {}
    profile_block = _resolve_qos_ref(full_xml, base_name)
    if not profile_block:
        return {}
    if tag == "data_writer":
        block = _extract_tag_block(profile_block, "datawriter_qos") or profile_block
    else:
        block = _extract_tag_block(profile_block, "datareader_qos") or profile_block
    return parse_profile(block)


def resolve_profile_for_block(
    full_xml: str, block: EntityBlock, dds: str = "connext",
    _visited: set[str] | None = None,
) -> Tuple[Dict[str, str | list[str]], str, Dict[str, str], List[str]]:
    """
    Merge QoS for single entity block. Resolve base_name inheritance.
    Returns entity_xml as resolved content so rules can parse deadline/partition.
    """
    source_info: Dict[str, str] = {}
    log_msgs: List[str] = []
    base_qos: Dict[str, str | list[str]] = {}
    visited = _visited if _visited is not None else set()
    entity_xml = block.block_xml

    # Resolve base_name (Library::Profile or Profile) - may be only source of QoS
    resolved_block: str | None = None
    if block.base_profile_name and block.base_profile_name not in visited:
        visited.add(block.profile_name or block.base_profile_name or "anon")
        parent_qos = _resolve_base_profile(
            full_xml, block.base_profile_name, block.tag
        )
        for k, v in parent_qos.items():
            if v and (isinstance(v, list) or str(v).strip()):
                base_qos[k] = v
                source_info[k] = f"base:{block.base_profile_name}"
        # Get resolved XML for rules (deadline, partition, etc.)
        profile_block = _resolve_qos_ref(full_xml, block.base_profile_name)
        if profile_block:
            resolved_block = (
                _extract_tag_block(profile_block, "datawriter_qos")
                if block.tag == "data_writer"
                else _extract_tag_block(profile_block, "datareader_qos")
            )

    # Entity block QoS (inline overrides)
    entity_qos = parse_profile(block.block_xml)
    has_inline = any(
        k != "partition_list"
        and v
        and (isinstance(v, list) or str(v).strip())
        for k, v in entity_qos.items()
    )
    for k, v in entity_qos.items():
        if v and (isinstance(v, list) or str(v).strip()):
            base_qos[k] = v
            source_info[k] = "Connext_Entity"

    # Use resolved XML for rules when block has only reference (no inline QoS)
    if resolved_block and not has_inline:
        entity_xml = resolved_block

    return base_qos, entity_xml, source_info, log_msgs


def get_topic_profile_qos(
    full_xml: str, topic_name: str | None
) -> Dict[str, str | list[str]] | None:
    """RTI does not have <topic profile_name="...">. Return None."""
    return None


def resolve_profile(
    xml: str, role: str, dds: str = "connext"
) -> Tuple[Dict[str, str | list[str]], str, Dict[str, str], List[str]]:
    """
    For XML pair mode: resolve pub/sub from single XML.
    Uses first/default qos_profile with datawriter_qos or datareader_qos.
    """
    profiles = _find_qos_profile_blocks(xml)
    if not profiles:
        return parse_profile(xml), xml, {}, []

    default = next((p for p in profiles if p[2]), profiles[0])
    profile_name, block, is_default, base_name = default

    tag = "data_writer" if role == "pub" else "data_reader"
    entity_block = (
        _extract_tag_block(block, "datawriter_qos")
        if role == "pub"
        else _extract_tag_block(block, "datareader_qos")
    )
    if not entity_block:
        entity_block = block

    eb = EntityBlock(
        tag=tag,
        profile_name=profile_name,
        topic_name=None,
        block_xml=entity_block,
        is_default=is_default,
        base_profile_name=base_name,
    )
    return resolve_profile_for_block(xml, eb, dds)
