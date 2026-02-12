#!/usr/bin/env python3
"""
Fast DDS QoS profile XML parser.

Parses eProsima Fast DDS (Fast RTPS) XML schema.

5-level priority (rmw_fastrtps official):
  L1: Code (rclcpp::QoS non-DEFAULT) - Highest priority
  L2: <topic profile_name="/topic"> - Topic name auto-matching
  L3: <data_writer/reader profile_name="/topic"> - Jazzy style topic matching
  L4: <publisher/subscriber profile_name="..."> - Humble, when code-specified
  L5: is_default_profile="true" - Fallback
"""
import re
from dataclasses import dataclass
from typing import Dict, List, Tuple

# ────────── Source level constants ──────────
LEVEL_CODE = "Level_1_Code"
LEVEL_TOPIC = "Level_2_Topic"
LEVEL_DATA_WRITER = "Level_3_DataWriter"
LEVEL_DATA_READER = "Level_3_DataReader"
LEVEL_PUBLISHER = "Level_4_Publisher"
LEVEL_SUBSCRIBER = "Level_4_Subscriber"
LEVEL_DEFAULT = "Level_5_Default"

# ────────── Entity block types ──────────
ENTITY_TAGS_PUB = ("publisher", "data_writer")
ENTITY_TAGS_SUB = ("subscriber", "data_reader")
ENTITY_TAGS_ALL = ENTITY_TAGS_PUB + ENTITY_TAGS_SUB + ("topic",)


@dataclass
class EntityBlock:
    """Single profile block in file (publisher, subscriber, data_writer, data_reader, topic)."""

    tag: str  # publisher | subscriber | data_writer | data_reader | topic
    profile_name: str
    topic_name: str | None  # Entity: inline <topic><name>, topic: profile_name
    block_xml: str
    is_default: bool
    base_profile_name: str | None = None  # Fast DDS base_profile_name inheritance


# ────────── Profile block extraction ──────────
_PROFILE_TAGS = ("topic", "publisher", "subscriber", "data_writer", "data_reader")
_TOPIC_LEVEL_KEYS = (
    "history", "history_depth", "max_samples", "max_instances", "max_samples_per_instance"
)


def _find_tag_blocks(xml: str, tag: str) -> List[Tuple[str, str, bool, str | None]]:
    """Extract <tag profile_name="X" base_profile_name="Y" is_default_profile="true">...</tag> blocks from XML."""
    results: List[Tuple[str, str, bool, str | None]] = []
    pattern = rf"<\s*{tag}\s+([^>]*)>"
    for m in re.finditer(pattern, xml, re.I):
        attrs = m.group(1)
        profile_m = re.search(r'profile_name\s*=\s*["\']([^"\']+)["\']', attrs, re.I)
        default_m = re.search(r'is_default_profile\s*=\s*["\']?(true|false)["\']?', attrs, re.I)
        base_m = re.search(r'base_profile_name\s*=\s*["\']([^"\']+)["\']', attrs, re.I)
        profile_name = profile_m.group(1).strip() if profile_m else ""
        is_default = default_m and default_m.group(1).lower() == "true"
        base_profile_name = base_m.group(1).strip() if base_m else None

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
                    results.append((profile_name, block, is_default, base_profile_name))
                    break
                i = close_pos + len(close_m.group(0))
            elif open_m:
                depth += 1
                i = open_pos + len(open_m.group(0))
            else:
                break
    return results


def _extract_inline_topic_name(entity_xml: str) -> str | None:
    """Extract <name> from inline <topic> block within entity."""
    topic_m = re.search(r"<\s*topic\s*>([\s\S]*?)<\s*/\s*topic\s*>", entity_xml, re.I)
    if not topic_m:
        return None
    name_m = re.search(r"<\s*name\s*>([^<]+)</name\s*>", topic_m.group(1), re.I)
    return name_m.group(1).strip() if name_m else None


def extract_profiles(xml: str) -> Dict[str, List[Tuple[str, str, bool, str | None]]]:
    """Extract all profile blocks from XML. Tuple: (profile_name, block_xml, is_default, base_profile_name)."""
    out: Dict[str, List[Tuple[str, str, bool, str | None]]] = {}
    for tag in _PROFILE_TAGS:
        blocks = _find_tag_blocks(xml, tag)
        if blocks:
            out[tag] = blocks
    return out


def extract_all_entity_blocks(xml: str) -> List[EntityBlock]:
    """Extract all publisher, subscriber, data_writer, data_reader, topic blocks."""
    result: List[EntityBlock] = []
    for tag in ENTITY_TAGS_ALL:
        blocks = _find_tag_blocks(xml, tag)
        for profile_name, block_xml, is_default, base_profile_name in blocks:
            if tag == "topic":
                topic_name = profile_name or None
            else:
                topic_name = _extract_inline_topic_name(block_xml)
            result.append(
                EntityBlock(
                    tag=tag,
                    profile_name=profile_name or "",
                    topic_name=topic_name,
                    block_xml=block_xml,
                    is_default=is_default,
                    base_profile_name=base_profile_name,
                )
            )
    return result


def _find_parent_block(
    profiles: Dict[str, List[Tuple[str, str, bool, str | None]]],
    tag: str,
    base_profile_name: str,
) -> Tuple[str, str, bool, str | None] | None:
    """Return parent block where profile_name matches base_profile_name in same tag."""
    entity_tags = ENTITY_TAGS_PUB + ENTITY_TAGS_SUB
    # publisher <-> data_writer, subscriber <-> data_reader mapping
    tag_to_check = [tag]
    if tag == "publisher":
        tag_to_check = ["publisher", "data_writer"]
    elif tag == "data_writer":
        tag_to_check = ["data_writer", "publisher"]
    elif tag == "subscriber":
        tag_to_check = ["subscriber", "data_reader"]
    elif tag == "data_reader":
        tag_to_check = ["data_reader", "subscriber"]
    for t in tag_to_check:
        blocks = profiles.get(t, [])
        for b in blocks:
            if b[0] == base_profile_name:
                return b
    return None


def resolve_profile_for_block(
    full_xml: str, block: EntityBlock, dds: str = "fast",
    _visited: set[str] | None = None,
) -> Tuple[Dict[str, str | list[str]], str, Dict[str, str], List[str]]:
    """
    Merge single entity block QoS in 5-level hierarchy.
    For base_profile_name inheritance, load parent QoS first then child overrides.

    Returns
    -------
    tuple
        (qos_dict, entity_xml, source_info, log_msgs)

    """
    if block.tag == "topic":
        q = parse_profile(block.block_xml)
        src = {k: LEVEL_TOPIC for k in q if q.get(k)}
        return q, block.block_xml, src, []

    source_info: Dict[str, str] = {}
    log_msgs: List[str] = []

    topic_name = block.topic_name
    # Jazzy: data_writer profile_name="/cmd_vel" -> use as topic
    if block.tag in ("data_writer", "data_reader") and not topic_name and block.profile_name:
        if block.profile_name.startswith("/") or _normalize_topic_for_match(block.profile_name):
            topic_name = (
                block.profile_name
                if block.profile_name.startswith("/")
                else f"/{block.profile_name}"
            )

    # base_profile_name inheritance: merge parent QoS first (prevent circular reference)
    base_qos: Dict[str, str | list[str]] = {}
    visited = _visited if _visited is not None else set()
    if block.base_profile_name and block.base_profile_name not in visited:
        profiles = extract_profiles(full_xml)
        parent = _find_parent_block(profiles, block.tag, block.base_profile_name)
        if parent:
            pname, pblock, pdefault, _ = parent
            parent_block = EntityBlock(
                tag=block.tag,
                profile_name=pname,
                topic_name=_extract_inline_topic_name(pblock),
                block_xml=pblock,
                is_default=pdefault,
                base_profile_name=parent[3],
            )
            visited.add(block.profile_name or block.base_profile_name or "anonymous")
            parent_qos, _, _, _ = resolve_profile_for_block(
                full_xml, parent_block, dds, _visited=visited
            )
            base_qos = dict(parent_qos)
            for k in list(base_qos.keys()):
                if base_qos.get(k):
                    source_info[k] = f"base:{block.base_profile_name}"

    # L5: default
    if block.is_default:
        base_qos = _parse_entity_qos(block.block_xml)
        for k in list(base_qos.keys()):
            if base_qos.get(k):
                source_info[k] = LEVEL_DEFAULT

    # L4 (publisher/subscriber) or L3 (data_writer/reader)
    entity_qos = _parse_entity_qos(block.block_xml)
    entity_level = (
        LEVEL_PUBLISHER if block.tag == "publisher" else
        LEVEL_SUBSCRIBER if block.tag == "subscriber" else
        LEVEL_DATA_WRITER if block.tag == "data_writer" else LEVEL_DATA_READER
    )
    _merge_qos(base_qos, entity_qos, entity_level, source_info)

    # L2: topic overlay
    profiles = extract_profiles(full_xml)
    topic_profiles = profiles.get("topic", [])
    selected_topic_block: str | None = None
    topic_profile_name: str = ""
    if topic_name:
        for pname, pblock, *_ in topic_profiles:
            if _topic_profile_matches(pname, topic_name):
                selected_topic_block = pblock
                topic_profile_name = pname or topic_name
                break
        if not selected_topic_block:
            default_topic = next((b for b in topic_profiles if b[2]), None)
            if default_topic:
                selected_topic_block = default_topic[1]
                topic_profile_name = default_topic[0] or "default"
    else:
        default_topic = next((b for b in topic_profiles if b[2]), None)
        if default_topic:
            selected_topic_block = default_topic[1]
            topic_profile_name = default_topic[0] or "default"

    if selected_topic_block:
        topic_qos = parse_profile(selected_topic_block)
        had_entity_override = any(
            topic_qos.get(k) and source_info.get(k) in (
                LEVEL_DATA_WRITER, LEVEL_DATA_READER, LEVEL_PUBLISHER, LEVEL_SUBSCRIBER
            )
            for k in _TOPIC_LEVEL_KEYS
        )
        for k in _TOPIC_LEVEL_KEYS:
            if topic_qos.get(k):
                base_qos[k] = topic_qos[k]
                source_info[k] = LEVEL_TOPIC
        if had_entity_override:
            display_topic = topic_profile_name or (topic_name or "?")
            log_msgs.append(
                f'[INFO] Applied XML topic profile "{display_topic}" overriding '
                f"{block.tag} profile"
            )

    return base_qos, block.block_xml, source_info, log_msgs


# ────────── Tag extraction patterns ──────────
TAG_PATTERNS = {
    "reliability": re.compile(
        r"<\s*reliability\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*reliability\s*>",
        re.I | re.S,
    ),
    "history": re.compile(
        r"<\s*historyQos\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*historyQos\s*>",
        re.I | re.S,
    ),
    "history_depth": re.compile(
        r"<\s*historyQos\s*>.*?<\s*depth\s*>(\d+)</depth>", re.I | re.S
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
        r"<\s*destinationOrder\s*>.*?<\s*kind\s*>(\w+)\s*</kind\s*>.*?</\s*destinationOrder\s*>",
        re.I | re.S,
    ),
    "max_samples": re.compile(
        r"<\s*resourceLimitsQos\s*>.*?<\s*max_samples\s*>(\d+)</max_samples>",
        re.I | re.S,
    ),
    "max_instances": re.compile(
        r"<\s*resourceLimitsQos\s*>.*?<\s*max_instances\s*>(\d+)</max_instances>",
        re.I | re.S,
    ),
    "max_samples_per_instance": re.compile(
        r"<\s*resourceLimitsQos\s*>.*?<\s*max_samples_per_instance\s*>(\d+)"
        r"</max_samples_per_instance>",
        re.I | re.S,
    ),
    "autodispose": re.compile(
        r"<\s*writerDataLifecycle\s*>.*?"
        r"<\s*autodispose_unregistered_instances\s*>\s*(\w+)\s*"
        r"</autodispose_unregistered_instances\s*>.*?"
        r"</\s*writerDataLifecycle\s*>",
        re.I | re.S,
    ),
    "autoenable": re.compile(
        r"<\s*autoenable_created_entities\s*>\s*(\w+)\s*</autoenable_created_entities\s*>",
        re.I | re.S,
    ),
    "liveliness": re.compile(
        r"<\s*liveliness\s*>.*?<\s*kind\s*>(.*?)</kind\s*>.*?</\s*liveliness\s*>",
        re.I | re.S,
    ),
    "nowriter_sec_r": re.compile(
        r"<\s*readerDataLifecycle\s*>.*?"
        r"<\s*autopurge_nowriter_samples_delay\s*>.*?"
        r"<\s*sec\s*>([^<]+)</sec\s*>",
        re.I | re.S,
    ),
    "nowriter_nsec_r": re.compile(
        r"<\s*readerDataLifecycle\s*>.*?"
        r"<\s*autopurge_nowriter_samples_delay\s*>.*?"
        r"<\s*nanosec\s*>([^<]+)</nanosec\s*>",
        re.I | re.S,
    ),
    "userdata": re.compile(
        r"<\s*userData\s*>.*?<\s*value\s*>([^<]+)</value\s*>", re.I | re.S
    ),
    "autopurge_disposed_samples_delay": re.compile(
        r"<\s*readerDataLifecycle\s*>.*?"
        r"<\s*autopurge_disposed_samples_delay\s*>.*?"
        r"<\s*sec\s*>(\d+)</sec\s*>",
        re.I | re.S,
    ),
}

# ────────── DEADLINE helpers ──────────
# [^<]+: capture numbers, DURATION_INFINITY, 4294967295, etc.
DEADLINE_RE = re.compile(
    r"<\s*deadline\s*>[^<]*?<\s*period\s*>"
    r"(?:[^<]*?<\s*sec\s*>([^<]+)\s*</sec\s*>)?"
    r"(?:[^<]*?<\s*nanosec\s*>([^<]+)\s*</nanosec\s*>)?",
    re.I | re.S,
)

# ────────── LIVELINESS helpers ──────────
LEASE_RE = re.compile(
    r"<\s*liveliness\s*>.*?<\s*lease_duration\s*>"
    r"(?:[^<]*?<\s*sec\s*>([^<]+)\s*</sec\s*>)?"
    r"(?:[^<]*?<\s*nanosec\s*>([^<]+)\s*</nanosec\s*>)?",
    re.I | re.S,
)

ANNOUNCE_RE = re.compile(
    r"<\s*liveliness\s*>.*?<\s*announcement_period\s*>"
    r"(?:[^<]*?<\s*sec\s*>([^<]+)</sec\s*>)?"
    r"(?:[^<]*?<\s*nanosec\s*>([^<]+)</nanosec\s*>)?",
    re.I | re.S,
)

# ────────── LIFESPAN helpers ──────────
LIFESPAN_RE = re.compile(
    r"<\s*lifespan\s*>.*?</\s*lifespan\s*>",
    re.I | re.S,
)

# ────────── partition helpers ──────────
PART_ALL_RE = re.compile(
    r"<\s*partition\s*>.*?</\s*partition\s*>", re.I | re.S
)
NAME_RE = re.compile(r"<\s*name\s*>([^<]+)</name\s*>", re.I | re.S)

INF_SET = {"DURATION_INFINITY", "4294967295"}


def parse_duration_field(txt: str | None) -> int | None:
    """Return None if infinite (INF), otherwise return integer."""
    if not txt:
        return 0
    t = txt.strip().upper()
    if t in INF_SET:
        return None
    return int(t)


def is_inf(txt: str | None) -> bool:
    """Check if text represents infinite value."""
    return txt and txt.strip().upper() in INF_SET


def deadline_enabled(xml: str) -> bool:
    """Check if DEADLINE is configured."""
    m = DEADLINE_RE.search(xml)
    if not m:
        return False
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return True
    return (sec_val != 0) or (nsec_val != 0)


def deadline_period_ns(xml: str) -> int | None:
    """Return DEADLINE period as ns integer. Return None if not set/infinite."""
    if not xml or not isinstance(xml, str):
        return None
    m = DEADLINE_RE.search(xml)
    if not m:
        return None
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return None
    return sec_val * 1_000_000_000 + nsec_val


def lease_duration_ns(xml: str) -> int | None:
    """Return liveliness lease_duration in ns. None if infinite."""
    m = LEASE_RE.search(xml)
    if not m:
        return None
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return None
    return sec_val * 1_000_000_000 + nsec_val


def announcement_period_ns(xml: str) -> int | None:
    """Return liveliness announcement_period in ns."""
    m = ANNOUNCE_RE.search(xml)
    if not m:
        return None
    sec = parse_duration_field(m.group(1))
    nsec = parse_duration_field(m.group(2))
    if sec is None or nsec is None:
        return None
    return sec * 1_000_000_000 + nsec


def partition_list(xml: str) -> list[str]:
    """Extract partition name list from XML."""
    if not xml or not isinstance(xml, str):
        return [""]
    blk = PART_ALL_RE.search(xml)
    if not blk:
        return [""]
    names = NAME_RE.findall(blk.group(0))
    return [n.strip() for n in names] or [""]


def parse_profile(xml: str) -> Dict[str, str | list[str]]:
    """Parse QoS profile from XML string and return as dictionary."""
    out: Dict[str, str | list[str]] = {}
    for k, pat in TAG_PATTERNS.items():
        m = pat.search(xml)
        out[k] = (m.group(1).upper() if m else "")
    out["partition_list"] = partition_list(xml)
    return out


def _parse_entity_qos(entity_xml: str) -> Dict[str, str | list[str]]:
    """Parse <qos> + inline <topic> merge from entity blocks."""
    qos_block = ""
    topic_m = re.search(r"<\s*topic\s*>([\s\S]*?)<\s*/\s*topic\s*>", entity_xml, re.I)
    qos_m = re.search(r"<\s*qos\s*>([\s\S]*?)<\s*/\s*qos\s*>", entity_xml, re.I)
    if topic_m:
        qos_block += topic_m.group(0)
    if qos_m:
        qos_block += qos_m.group(0)
    return parse_profile(qos_block) if qos_block else parse_profile(entity_xml)


def _normalize_topic_for_match(name: str | None) -> str:
    """Normalization for topic matching: /cmd_vel, cmd_vel -> cmd_vel."""
    if not name:
        return ""
    return (name or "").strip().strip("/") or ""


def _topic_matches(profile_name: str, topic_name: str | None) -> bool:
    """Jazzy: Check if profile_name matches topic name (L3 auto matching)."""
    if not topic_name:
        return False
    p = _normalize_topic_for_match(profile_name)
    t = _normalize_topic_for_match(topic_name)
    if not p or not t:
        return False
    return p == t or f"/{p}" == f"/{t}"


def _topic_profile_matches(pname: str, topic_name: str | None) -> bool:
    """Check if L2 topic profile matches topic_name."""
    if _topic_matches(pname, topic_name) or pname == topic_name:
        return True
    return pname == f"/{topic_name or ''}"


def get_topic_profile_qos(full_xml: str, topic_name: str | None) -> Dict[str, str | list[str]] | None:
    """
    Return QoS from XML <topic profile_name="..."> block matching topic_name.
    Used for L2 application when code-only entities match XML topic profile.
    """
    if not topic_name:
        return None
    profiles = extract_profiles(full_xml)
    topic_profiles = profiles.get("topic", [])
    for pname, pblock, *_ in topic_profiles:
        if _topic_profile_matches(pname, topic_name):
            return parse_profile(pblock)
    default_topic = next((b for b in topic_profiles if b[2]), None)
    if default_topic:
        return parse_profile(default_topic[1])
    return None


def _merge_qos(
    base: Dict[str, str | list[str]],
    overlay: Dict[str, str | list[str]],
    level: str,
    source_info: Dict[str, str],
    keys: tuple[str, ...] | None = None,
) -> None:
    """Merge overlay into base. Record source in source_info."""
    target_keys = keys or list(overlay.keys())
    for k in target_keys:
        v = overlay.get(k)
        if v and (isinstance(v, list) or str(v).strip()):
            base[k] = v
            source_info[k] = level


def resolve_profile(
    xml: str, role: str, dds: str = "fast"
) -> Tuple[Dict[str, str | list[str]], str, Dict[str, str], List[str]]:
    """
    Merge QoS with 5-level hierarchy. Apply Full Override.

    Returns
    -------
    tuple
        (qos_dict, entity_xml, source_info, log_msgs)

    """
    profiles = extract_profiles(xml)
    source_info: Dict[str, str] = {}
    log_msgs: List[str] = []

    entity_tag = "data_writer" if role == "pub" else "data_reader"
    fallback_tag = "publisher" if role == "pub" else "subscriber"

    # Entity block selection: prioritize data_writer/reader, otherwise publisher/subscriber
    entity_blocks = profiles.get(entity_tag) or profiles.get(fallback_tag)
    used_tag = entity_tag if profiles.get(entity_tag) else fallback_tag

    if not entity_blocks:
        return parse_profile(xml), xml, source_info, log_msgs

    # Select first block or default
    profile_name, entity_xml, is_default_entity, _ = entity_blocks[0]
    if len(entity_blocks) > 1:
        default_entity = next((b for b in entity_blocks if b[2]), entity_blocks[0])
        profile_name, entity_xml, is_default_entity, _ = default_entity

    topic_name = _extract_inline_topic_name(entity_xml)
    # Jazzy: if data_writer profile_name="/cmd_vel" format, use as topic
    if used_tag in ("data_writer", "data_reader") and not topic_name and profile_name:
        if profile_name.startswith("/") or _normalize_topic_for_match(profile_name):
            topic_name = profile_name if profile_name.startswith("/") else f"/{profile_name}"

    # L5: default entity base
    base_qos: Dict[str, str | list[str]] = {}
    if is_default_entity:
        base_qos = _parse_entity_qos(entity_xml)
        for k in list(base_qos.keys()):
            if base_qos.get(k):
                source_info[k] = LEVEL_DEFAULT

    # L4 (publisher/subscriber) or L3 (data_writer/reader) entity
    entity_qos = _parse_entity_qos(entity_xml)
    entity_level = (
        LEVEL_PUBLISHER if used_tag == "publisher" else
        LEVEL_SUBSCRIBER if used_tag == "subscriber" else
        LEVEL_DATA_WRITER if used_tag == "data_writer" else LEVEL_DATA_READER
    )
    _merge_qos(base_qos, entity_qos, entity_level, source_info)

    # L2: topic profile overlay (Full Override)
    topic_profiles = profiles.get("topic", [])
    selected_topic_block: str | None = None
    topic_profile_name: str = ""
    if topic_name:
        for pname, pblock, *_ in topic_profiles:
            if _topic_profile_matches(pname, topic_name):
                selected_topic_block = pblock
                topic_profile_name = pname or topic_name
                break
        if not selected_topic_block:
            default_topic = next((b for b in topic_profiles if b[2]), None)
            if default_topic:
                selected_topic_block = default_topic[1]
                topic_profile_name = default_topic[0] or "default"

    if not selected_topic_block:
        default_topic = next((b for b in topic_profiles if b[2]), None)
        if default_topic:
            selected_topic_block = default_topic[1]
            topic_profile_name = default_topic[0] or "default"

    if selected_topic_block:
        topic_qos = parse_profile(selected_topic_block)
        had_entity_override = any(
            topic_qos.get(k) and source_info.get(k) in (
                LEVEL_DATA_WRITER, LEVEL_DATA_READER, LEVEL_PUBLISHER, LEVEL_SUBSCRIBER
            )
            for k in _TOPIC_LEVEL_KEYS
        )
        for k in _TOPIC_LEVEL_KEYS:
            if topic_qos.get(k):
                base_qos[k] = topic_qos[k]
                source_info[k] = LEVEL_TOPIC
        if had_entity_override:
            display_topic = topic_profile_name or (topic_name or "?")
            log_msgs.append(
                f'[INFO] Applied XML topic profile "{display_topic}" overriding '
                f"{used_tag} profile"
            )

    return base_qos, entity_xml, source_info, log_msgs
