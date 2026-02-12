#!/usr/bin/env python3
"""
Output module.

Centralizes all terminal output (except CLI error/usage).
Format and print check results, info, and warnings.
Collects per-node buffer, sorts alphabetically, adds line break on node change.
"""
import re
import shutil
from pathlib import Path
from typing import List, Tuple

# Topic column width: ~35% of terminal, min 20, default 35 if terminal size unknown
def _topic_column_width() -> int:
    try:
        w = shutil.get_terminal_size().columns
    except (OSError, AttributeError):
        w = 80
    return max(20, int(w * 0.35))

# Buffer: (topic, node, "info"|"warning", payload) - output per topic: info → warning order
_output_buffer: list[tuple[str, str, str, object]] = []

# ────────── ANSI color codes ──────────
RED = "\033[31m"
BLUE = "\033[34m"
YELLOW = "\033[33m"
PURPLE = "\033[35m"
RESET = "\033[0m"

SEVERITY_COLOR = {
    "Structural": RED,
    "Functional": YELLOW,
    "Operational": PURPLE,
    "Warn": "\033[37m",
}


def color(txt: str, c: str) -> str:
    """Apply ANSI color to text."""
    return f"{c}{txt}{RESET}"


def format_warning(severity: str, side: str, msg: str) -> str:
    """Format a single warning for display."""
    tag = f"[{severity.upper()}]"
    color_tag = color(tag, SEVERITY_COLOR.get(severity, RED))
    if side == "CROSS":
        return f"{color_tag} {msg}"
    side_tag = color(f"[{side}]", BLUE)
    return f"{color_tag} {side_tag} {msg}"


def format_warning_with_node(severity: str, side: str, msg: str, node: str) -> str:
    """Format warning with [SEVERITY] [node] prefix."""
    tag = f"[{severity.upper()}]"
    color_tag = color(tag, SEVERITY_COLOR.get(severity, RED))
    node_part = f" [{node}]" if (node or "").strip() else ""
    if side == "CROSS":
        return f"{color_tag}{node_part} {msg}"
    side_tag = color(f"[{side}]", BLUE)
    return f"{color_tag}{node_part} {side_tag} {msg}"


def _topic_display(topic: str) -> str:
    """Topic display: replace custom_topic_name with *."""
    t = (topic or "").strip().strip("/") or "?"
    if "custom_topic_name" in t:
        # /controller_selector_custom_topic_name -> /controller_selector_*
        t = t.replace("custom_topic_name", "*")
    return f"/{t}"


def buffer_info(node: str, line: str, topic: str = "") -> None:
    """Add info line to buffer. Grouped by topic (topic → warnings order)."""
    _output_buffer.append((topic or "", node or "", "info", line))


def _normalize_qos_msg(msg: str) -> str:
    """Invalid QoS: / QoS warning: → convert to line break + QoS Conflict:."""
    msg = msg.replace("Invalid QoS:", "\nQoS Conflict:", 1)
    msg = msg.replace("QoS warning:", "\nQoS Conflict:", 1)
    return msg


def _abbreviate_qos_msg(msg: str) -> str:
    """Abbreviate warning message: remove Invalid QoS:/QoS warning:/QoS Conflict: prefixes."""
    for prefix in ("Invalid QoS: ", "QoS warning: ", "QoS Conflict: "):
        if msg.startswith(prefix):
            return msg[len(prefix):].strip()
        if "\nQoS Conflict: " in msg:
            return msg.split("\nQoS Conflict: ", 1)[-1].strip()
    return msg.strip()


def buffer_warning(
    pub_node: str,
    sub_node: str,
    severity: str,
    side: str,
    topic: str | None,
    msg: str,
) -> None:
    """Add warning to buffer. Output per topic: immediately below info."""
    pn = (pub_node or "").strip()
    full_msg = _abbreviate_qos_msg(msg)
    _output_buffer.append((topic or "", pn or "", "warning", (severity, side, full_msg)))


def buffer_qos_sources(
    topic: str,
    pub_sum: str,
    sub_sum: str,
    pub_had_override: bool,
    sub_had_override: bool,
    pub_entity_tag: str,
    sub_entity_tag: str,
    pub_node: str = "",
    sub_node: str = "",
) -> None:
    """Add same content as print_qos_sources to buffer."""
    line = _format_qos_sources_line(
        topic, pub_sum, sub_sum,
        pub_had_override, sub_had_override,
        pub_entity_tag, sub_entity_tag,
        pub_node, sub_node,
    )
    buffer_info((pub_node or "").strip(), line, topic=topic)


def buffer_orphan_topic(
    topic: str, side: str, node_name: str = "", level: str = "L1"
) -> None:
    """Pub-only/Sub-only topic display. Topic:[/X] Pub/Sub[node][level] format. custom_topic_name → *."""
    node = (node_name or "").strip()
    topic_norm = _topic_display(topic)
    role = "Pub" if (side or "").upper() == "PUB" else "Sub"
    topic_part = f"Topic:[{topic_norm}]"
    topic_width = _topic_column_width()
    topic_padded = topic_part.ljust(topic_width)
    line = f"{topic_padded}  {role}[{node or '?'}][{level}]"
    buffer_info(node, line, topic=topic)


def flush_output_buffer() -> None:
    """Sort buffer by node and output. Node info → corresponding warnings order."""
    global _output_buffer
    if not _output_buffer:
        return
    # Sort: node ascending, topic ascending, info first warning last (one node + warnings immediately below)
    type_order = {"info": 0, "warning": 1}
    sorted_items = sorted(
        _output_buffer,
        key=lambda x: ((x[1] or "").lower(), (x[0] or "").lower(), type_order.get(x[2], 1)),
    )
    _output_buffer = []
    prev_node: str = ""
    prev_topic: str | None = None
    for topic, node, line_type, payload in sorted_items:
        curr_topic = topic or ""
        curr_node = node or ""
        if prev_node != "" and curr_node != prev_node and curr_topic != prev_topic:
            print()
        prev_node = curr_node
        prev_topic = curr_topic
        if line_type == "info":
            print(f"[INFO] {payload}")
        else:
            severity, side, full_msg = payload
            print(format_warning_with_node(severity, side, full_msg, node))


def clear_output_buffer() -> None:
    """Clear buffer."""
    global _output_buffer
    _output_buffer = []


def print_info(msg: str, node: str = "") -> None:
    """Print [INFO] message. If node exists, use [INFO] [node] format."""
    prefix = f"[INFO] [{node}] " if (node or "").strip() else "[INFO] "
    print(f"{prefix}{msg}")


def print_warn(msg: str) -> None:
    """Print [WARN] message."""
    print(f"[WARN] {msg}")


def print_list_mode(xml_files: List[Path], package_path: Path) -> None:
    """Print list mode output (XML files in package)."""
    print(f"Found {len(xml_files)} XML file(s) in {package_path}")
    for p in xml_files:
        print(f"  {p}")


def print_cyclone_unsupported() -> None:
    """Print Cyclone DDS XML pair mode rejection message."""
    print_info(
        "Cyclone DDS does not support XML QoS profiles. Use package mode only."
    )


def print_no_entity_pairs(path: Path) -> None:
    """Print when no Entity pairs found."""
    print_info(f"No Entity pairs found in {path}")


def _level_to_short(sum_str: str) -> str:
    """Level_1_Code, Level_3_DataWriter -> L1 (minimum level = highest priority)."""
    nums = re.findall(r"Level_(\d+)_", sum_str)
    if not nums:
        return "Default"
    return f"L{min(int(n) for n in nums)}"


def _format_qos_sources_line(
    topic: str,
    pub_sum: str,
    sub_sum: str,
    pub_had_override: bool,
    sub_had_override: bool,
    pub_entity_tag: str,
    sub_entity_tag: str,
    pub_node: str = "",
    sub_node: str = "",
) -> str:
    """QoS source display: Topic:[/X] Pub[node][L] <-> Sub[node][L]"""
    def _format_side(sum_str: str, had_override: bool, entity_tag: str) -> str:
        short = _level_to_short(sum_str)
        if not had_override:
            return short
        t = (entity_tag or "").lower()
        if t in ("data_writer", "data_reader"):
            return "L3→L2"
        if t in ("publisher", "subscriber"):
            return "L4→L2"
        return f"{short}→L2"

    pub_display = _format_side(pub_sum, pub_had_override, pub_entity_tag)
    sub_display = _format_side(sub_sum, sub_had_override, sub_entity_tag)
    topic_norm = _topic_display(topic)
    pn = (pub_node or "").strip()
    sn = (sub_node or "").strip()
    topic_part = f"Topic:[{topic_norm}]"
    topic_width = _topic_column_width()
    topic_padded = topic_part.ljust(topic_width)
    return f"{topic_padded}  Pub[{pn or '?'}][{pub_display}]  <->  Sub[{sn or '?'}][{sub_display}]"


def print_qos_sources(
    topic: str,
    pub_sum: str,
    sub_sum: str,
    pub_had_override: bool,
    sub_had_override: bool,
    pub_entity_tag: str,
    sub_entity_tag: str,
    pub_node: str = "",
    sub_node: str = "",
) -> None:
    """Print QoS source levels for a pair. Show ignored levels on override (Lx→L2)."""
    line = _format_qos_sources_line(
        topic, pub_sum, sub_sum,
        pub_had_override, sub_had_override,
        pub_entity_tag, sub_entity_tag,
        pub_node, sub_node,
    )
    print_info(line, (pub_node or "").strip())


def print_warnings(
    warnings: List[Tuple[str, str, str | None, str, str, str]]
) -> None:
    """Print all rule violation warnings.
    Each item: (severity, side, topic, msg, pub_node, sub_node).
    pub_node/sub_node optional for backward compat (empty str).
    """
    for i, row in enumerate(warnings):
        if i > 0:
            print()
        severity = row[0]
        side = row[1]
        topic = row[2]
        msg = row[3]
        pub_node = row[4] if len(row) > 4 else ""
        sub_node = row[5] if len(row) > 5 else ""
        pn = (pub_node or "").strip()
        sn = (sub_node or "").strip()
        node_part = f" Pub[{pn}] Sub[{sn}]" if (pn or sn) else ""
        full_msg = f"Topic:{topic}:{node_part} {msg}" if topic else msg
        full_msg = _normalize_qos_msg(full_msg)
        print(format_warning(severity, side, full_msg))


def print_orphan_topic(
    topic: str, side: str, node_name: str = ""
) -> None:
    """Pub-only or Sub-only topic display (rule check skipped)."""
    node_part = f" [{node_name}]" if (node_name or "").strip() else ""
    print_info(f"Topic:{topic}:{node_part} {side.upper()} only (rules skipped)")


def print_success() -> None:
    """Print success message when all rules pass."""
    print("All Entities are safe !")


def print_summary(
    package_name: str,
    node_count: int,
    topic_count: int,
    endpoint_count: int,
    structural_count: int,
    functional_count: int,
    operational_count: int,
) -> None:
    """Print summary table at the end of output."""
    print()
    rows = [
        ("Package", package_name),
        ("Nodes", str(node_count)),
        ("Topics", str(topic_count)),
        ("Endpoints", str(endpoint_count)),
        ("Structural", str(structural_count)),
        ("Functional", str(functional_count)),
        ("Operational", str(operational_count)),
    ]
    col_w = max(len(r[0]) for r in rows) + 2
    val_w = max(len(r[1]) for r in rows) + 2
    sep = "─" * (col_w + val_w + 4)
    print(sep)
    for lbl, val in rows:
        print(f"  {lbl:<{col_w}} {val}")
    print(sep)
