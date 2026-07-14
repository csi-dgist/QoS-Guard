#!/usr/bin/env python3
"""
3-stage multi-pattern topic extraction engine.

Tier 1 (Literal): create_publisher/subscription literals
Tier 2 (Parameter): declare_parameter defaults
Tier 3 (Constant): constexpr, const std::string, #define

Process extracted topics to enable matching with XML <topic profile_name="...">, entity profiles.
"""
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from . import code_scanner

# ────────── Tier 1: Literal (create_publisher/subscription) ──────────
_RE_CPP_STR = re.compile(r'["\']([^"\']*)["\']')
_RE_CPP_CREATE_PUB = re.compile(
    r"create_publisher\s*<[^>]+>\s*\(\s*([^,)]+)\s*,\s*([^)]+)\s*\)",
    re.I | re.S,
)
_RE_CPP_CREATE_SUB = re.compile(
    r"create_subscription\s*<[^>]+>\s*\(\s*([^,)]+)\s*,\s*([^,)]+)\s*,\s*[^)]+\)",
    re.I | re.S,
)
_RE_PY_CREATE_PUB = re.compile(
    r"create_publisher\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*([^)]+)\s*\)",
    re.I | re.S,
)
_RE_PY_CREATE_PUB_NO_QOS = re.compile(
    r"create_publisher\s*\(\s*[^,]+,\s*([^,)]+)\s*\)",
    re.I | re.S,
)
_RE_PY_CREATE_SUB = re.compile(
    r"create_subscription\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*[^,]+,\s*([^)]+)\s*\)",
    re.I | re.S,
)
_RE_PY_CREATE_SUB_NO_QOS = re.compile(
    r"create_subscription\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*[^,]+\)",
    re.I | re.S,
)
_RE_PY_TOPIC_KW = re.compile(
    r"topic\s*=\s*[\"']([^\"']*)[\"']",
    re.I,
)

# ────────── Tier 2: Parameter (declare_parameter) ──────────
# C++: declare_parameter("name", "/topic"), declare_or_get_parameter, etc.
_RE_CPP_DECLARE_PARAM = re.compile(
    r"(?:declare_parameter|declare_or_get_parameter)\s*"
    r"(?:<[^>]+>\s*)?"
    r"\(\s*[\"']([^\"']+)[\"']\s*,\s*[\"']([^\"']*)[\"']\s*\)",
    re.I | re.S,
)
# std::string{"vo_map"} or std::string("costmap_filter_info")
_RE_CPP_DECLARE_PARAM_STD = re.compile(
    r"(?:declare_parameter|declare_or_get_parameter)\s*"
    r"(?:<[^>]+>\s*)?"
    r"\(\s*[\"']([^\"']+)[\"']\s*,\s*std::string\s*[({\s]*[\"']([^\"']*)[\"']\s*[)}\s]*\s*\)",
    re.I | re.S,
)
_RE_CPP_DECLARE_PARAM_ALT = re.compile(
    r"(?:declare_parameter|declare_or_get_parameter)\s*"
    r"(?:<[^>]+>\s*)?"
    r"\(\s*[\"']([^\"']+)[\"']\s*,\s*std::string\s*\{\s*[\"']([^\"']*)[\"']\s*\}\s*\)",
    re.I | re.S,
)
# Dynamic parameter: declare_or_get_parameter(name_ + "." + "map_topic", std::string("map"))
# If first argument has topic-related suffix, use default as topic
_RE_CPP_DECLARE_PARAM_DYNAMIC = re.compile(
    r"(?:declare_parameter|declare_or_get_parameter)\s*"
    r"\(\s*[^)]*\+\s*[^)]*[\"']\.?(\w+)[\"']\s*[^)]*,\s*std::string\s*[({\s]*[\"']([^\"']*)[\"']",
    re.I | re.S,
)
# Python: declare_parameter("name", "/topic")
_RE_PY_DECLARE_PARAM = re.compile(
    r"declare_parameter\s*\(\s*[\"']([^\"']+)[\"']\s*,\s*[\"']([^\"']*)[\"']\s*\)",
    re.I | re.S,
)
_RE_PY_DECLARE_PARAM_ALT = re.compile(
    r"declare_parameter\s*\(\s*[\"']([^\"']+)[\"']\s*,\s*[\"']([^\"']*)[\"']\s*\)",
    re.I,
)
# Param names that suggest topic (map_topic, filter_info_topic, route_graph, etc.)
_TOPIC_PARAM_SUFFIXES = ("topic", "topic_name", "_topic", "graph", "info")

# ────────── Tier 3: Constant ──────────
# constexpr char* TOPIC = "/topic"; constexpr const char* TOPIC = "/topic";
_RE_CPP_CONSTEXPR = re.compile(
    r"(?:constexpr\s+)?(?:const\s+)?(?:char\s*\*\s*|const\s+char\s*\*\s*)"
    r"(\w+)\s*=\s*[\"']([^\"']*)[\"']\s*;",
    re.I | re.S,
)
# const std::string TOPIC = "/topic"; static const std::string TOPIC = "/topic";
_RE_CPP_CONST_STRING = re.compile(
    r"(?:static\s+)?const\s+std::string\s+(\w+)\s*=\s*[\"']([^\"']*)[\"']\s*;",
    re.I | re.S,
)
# #define TOPIC "/topic"
_RE_CPP_DEFINE = re.compile(
    r"#define\s+(\w+)\s+[\"']([^\"']*)[\"']",
    re.I,
)
# Python: TOPIC = "/topic" or topic = "/topic"
_RE_PY_CONST = re.compile(
    r"^(\w+)\s*=\s*[\"']([^\"']*)[\"']\s*$",
    re.M,
)

# ────────── QoS extraction (reuse code_scanner logic) ──────────
_RE_CPP_RELIABILITY = re.compile(
    r"\.reliability\s*\(\s*(?:rclcpp::)?ReliabilityPolicy\s*::\s*(\w+)\s*\)",
    re.I,
)
_RE_CPP_DURABILITY = re.compile(
    r"\.durability\s*\(\s*(?:rclcpp::)?DurabilityPolicy\s*::\s*(\w+)\s*\)",
    re.I,
)
_RE_CPP_KEEP_LAST = re.compile(r"\.keep_last\s*\(\s*(\d+)\s*\)", re.I)
_RE_CPP_KEEP_ALL = re.compile(r"\.keep_all\s*\(\s*\)", re.I)
_RE_PY_QOS_PROFILE = re.compile(
    r"QoSProfile\s*\(([^)]*)\)",
    re.I | re.S,
)
_RE_PY_RELIABILITY = re.compile(
    r"reliability\s*=\s*(?:ReliabilityPolicy\.)?(\w+)",
    re.I,
)
_RE_PY_DURABILITY = re.compile(
    r"durability\s*=\s*(?:DurabilityPolicy\.)?(\w+)",
    re.I,
)
_RE_PY_DEPTH = re.compile(r"depth\s*=\s*(\d+)", re.I)


def _normalize_topic(t: str) -> str:
    """Topic normalization: route_graph -> /route_graph, /cmd_vel -> /cmd_vel."""
    if not t or not t.strip():
        return ""
    t = t.strip().strip('"\'')
    if t and not t.startswith("/"):
        t = "/" + t
    return t


def _is_topic_like(s: str) -> bool:
    """Check if topic format (/, or common ROS topic patterns)."""
    if not s:
        return False
    s = s.strip()
    if len(s) < 2:
        return False
    if "\x1b" in s or "\\" in s[:2]:  # ANSI escape, escape seq
        return False
    if "/tmp" in s.lower() or s.lower().startswith("tmp/"):
        return False
    if s.startswith("/"):
        return not s.startswith("//") and s[1] != "/"
    # Infer topic from param name: map_topic, filter_info_topic, route_graph
    lower = s.lower()
    if any(x in lower for x in ("topic", "graph", "costmap", "map", "filter")):
        return True
    return "." not in s and " " not in s


def _extract_qos_from_content(content: str, start_pos: int = 0) -> dict[str, str]:
    """Extract QoS settings from file content (prioritize near relevant position)."""
    out: dict[str, str] = {
        "reliability": "",
        "durability": "",
        "history": "KEEP_LAST",
        "history_depth": "10",
    }
    # 2000 character context before/after
    ctx_start = max(0, start_pos - 1000)
    ctx_end = min(len(content), start_pos + 1000)
    ctx = content[ctx_start:ctx_end]

    def _map_rel(r: str) -> str:
        return "RELIABLE" if "RELIABLE" in (r or "").upper() else ""

    def _map_dur(d: str) -> str:
        u = (d or "").upper()
        if "TRANSIENT_LOCAL" in u:
            return "TRANSIENT_LOCAL"
        if "TRANSIENT" in u:
            return "TRANSIENT"
        if "PERSISTENT" in u:
            return "PERSISTENT"
        return ""

    for m in _RE_CPP_RELIABILITY.finditer(ctx):
        out["reliability"] = _map_rel(m.group(1))
    for m in _RE_CPP_DURABILITY.finditer(ctx):
        out["durability"] = _map_dur(m.group(1))
    for m in _RE_CPP_KEEP_LAST.finditer(ctx):
        out["history_depth"] = m.group(1)
        out["history"] = "KEEP_LAST"
    for m in _RE_CPP_KEEP_ALL.finditer(ctx):
        out["history"] = "KEEP_ALL"

    for m in _RE_PY_QOS_PROFILE.finditer(ctx):
        args = m.group(1)
        r = _RE_PY_RELIABILITY.search(args)
        if r:
            out["reliability"] = _map_rel(r.group(1))
        d = _RE_PY_DURABILITY.search(args)
        if d:
            out["durability"] = _map_dur(d.group(1))
        dep = _RE_PY_DEPTH.search(args)
        if dep:
            out["history_depth"] = dep.group(1)
        break

    return out


def _get_line_number(content: str, pos: int) -> int:
    """Character position -> line number."""
    return content.count("\n", 0, pos) + 1


def _extract_tier1_cpp(path: Path, content: str) -> list[dict]:
    """Tier 1: C++ create_publisher/subscription literals."""
    results: list[dict] = []
    seen: set[tuple[str, int]] = set()

    for m in _RE_CPP_CREATE_PUB.finditer(content):
        topic_str = m.group(1).strip()
        qos_expr = m.group(2).strip()
        lit = _RE_CPP_STR.search(topic_str)
        if lit:
            topic = _normalize_topic(lit.group(1))
            if topic and (topic, m.start()) not in seen:
                seen.add((topic, m.start()))
                qos = code_scanner._extract_cpp_qos_from_context(content, qos_expr)
                results.append({
                    "topic": topic,
                    "source_type": "Literal",
                    "file_path": str(path),
                    "line_number": _get_line_number(content, m.start()),
                    "entity_type": "pub",
                    "detected_qos": qos,
                })

    for m in _RE_CPP_CREATE_SUB.finditer(content):
        topic_str = m.group(1).strip()
        qos_expr = m.group(2).strip()
        lit = _RE_CPP_STR.search(topic_str)
        if lit:
            topic = _normalize_topic(lit.group(1))
            if topic and (topic, m.start()) not in seen:
                seen.add((topic, m.start()))
                qos = code_scanner._extract_cpp_qos_from_context(content, qos_expr)
                results.append({
                    "topic": topic,
                    "source_type": "Literal",
                    "file_path": str(path),
                    "line_number": _get_line_number(content, m.start()),
                    "entity_type": "sub",
                    "detected_qos": qos,
                })

    return results


def _extract_tier1_py(path: Path, content: str) -> list[dict]:
    """Tier 1: Python create_publisher/subscription literals."""
    results: list[dict] = []
    seen: set[tuple[str, int]] = set()

    for m in _RE_PY_CREATE_PUB.finditer(content):
        topic_str = m.group(1).strip()
        qos_expr = m.group(2).strip()
        lit = _RE_CPP_STR.search(topic_str)
        if lit:
            topic = _normalize_topic(lit.group(1))
            if topic and (topic, m.start()) not in seen:
                seen.add((topic, m.start()))
                qos = code_scanner._extract_py_qos_from_profile(content, qos_expr)
                results.append({
                    "topic": topic,
                    "source_type": "Literal",
                    "file_path": str(path),
                    "line_number": _get_line_number(content, m.start()),
                    "entity_type": "pub",
                    "detected_qos": qos,
                })

    for m in _RE_PY_CREATE_PUB_NO_QOS.finditer(content):
        topic_str = m.group(1).strip()
        lit = _RE_CPP_STR.search(topic_str)
        if lit:
            topic = _normalize_topic(lit.group(1))
            if topic and (topic, m.start()) not in seen:
                seen.add((topic, m.start()))
                results.append({
                    "topic": topic,
                    "source_type": "Literal",
                    "file_path": str(path),
                    "line_number": _get_line_number(content, m.start()),
                    "entity_type": "pub",
                    "detected_qos": {},
                })

    for m in _RE_PY_CREATE_SUB.finditer(content):
        topic_str = m.group(1).strip()
        qos_expr = m.group(2).strip()
        lit = _RE_CPP_STR.search(topic_str)
        if lit:
            topic = _normalize_topic(lit.group(1))
            if topic and (topic, m.start()) not in seen:
                seen.add((topic, m.start()))
                qos = code_scanner._extract_py_qos_from_profile(content, qos_expr)
                results.append({
                    "topic": topic,
                    "source_type": "Literal",
                    "file_path": str(path),
                    "line_number": _get_line_number(content, m.start()),
                    "entity_type": "sub",
                    "detected_qos": qos,
                })

    for m in _RE_PY_CREATE_SUB_NO_QOS.finditer(content):
        topic_str = m.group(1).strip()
        lit = _RE_CPP_STR.search(topic_str)
        if lit:
            topic = _normalize_topic(lit.group(1))
            if topic and (topic, m.start()) not in seen:
                seen.add((topic, m.start()))
                results.append({
                    "topic": topic,
                    "source_type": "Literal",
                    "file_path": str(path),
                    "line_number": _get_line_number(content, m.start()),
                    "entity_type": "sub",
                    "detected_qos": {},
                })

    for m in _RE_PY_TOPIC_KW.finditer(content):
        topic = _normalize_topic(m.group(1))
        if topic and (topic, m.start()) not in seen:
            seen.add((topic, m.start()))
            results.append({
                "topic": topic,
                "source_type": "Literal",
                "file_path": str(path),
                "line_number": _get_line_number(content, m.start()),
                "entity_type": "pub",
                "detected_qos": {},
            })

    return results


def _extract_tier2_cpp(path: Path, content: str) -> list[dict]:
    """Tier 2: C++ declare_parameter defaults."""
    results: list[dict] = []
    seen: set[tuple[str, int]] = set()

    for m in _RE_CPP_DECLARE_PARAM.finditer(content):
        param_name = m.group(1)
        default_val = m.group(2).strip()
        if not _is_topic_like(default_val):
            if not any(s in param_name.lower() for s in _TOPIC_PARAM_SUFFIXES):
                continue
        topic = _normalize_topic(default_val) if default_val else _normalize_topic(param_name)
        if not topic:
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Parameter",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "param_name": param_name,
            "detected_qos": qos,
        })

    for m in _RE_CPP_DECLARE_PARAM_ALT.finditer(content):
        param_name = m.group(1)
        default_val = m.group(2).strip()
        if not _is_topic_like(default_val):
            if not any(s in param_name.lower() for s in _TOPIC_PARAM_SUFFIXES):
                continue
        topic = _normalize_topic(default_val) if default_val else _normalize_topic(param_name)
        if not topic:
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Parameter",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "param_name": param_name,
            "detected_qos": qos,
        })

    for m in _RE_CPP_DECLARE_PARAM_STD.finditer(content):
        param_name = m.group(1)
        default_val = m.group(2).strip()
        if not _is_topic_like(default_val):
            if not any(s in param_name.lower() for s in _TOPIC_PARAM_SUFFIXES):
                continue
        topic = _normalize_topic(default_val) if default_val else _normalize_topic(param_name)
        if not topic:
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Parameter",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "param_name": param_name,
            "detected_qos": qos,
        })

    for m in _RE_CPP_DECLARE_PARAM_DYNAMIC.finditer(content):
        suffix = m.group(1)
        default_val = (m.group(2) if m.lastindex >= 2 else "").strip()
        if not default_val:
            continue
        if not any(s in (suffix or "").lower() for s in _TOPIC_PARAM_SUFFIXES):
            continue
        topic = _normalize_topic(default_val)
        if not topic:
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Parameter",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "param_name": f"*.{suffix}",
            "detected_qos": qos,
        })

    return results


def _extract_tier2_py(path: Path, content: str) -> list[dict]:
    """Tier 2: Python declare_parameter defaults."""
    results: list[dict] = []
    seen: set[tuple[str, int]] = set()

    for m in _RE_PY_DECLARE_PARAM.finditer(content):
        param_name = m.group(1)
        default_val = m.group(2).strip()
        if not _is_topic_like(default_val):
            if not any(s in param_name.lower() for s in _TOPIC_PARAM_SUFFIXES):
                continue
        topic = _normalize_topic(default_val) if default_val else _normalize_topic(param_name)
        if not topic:
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Parameter",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "param_name": param_name,
            "detected_qos": qos,
        })

    return results


def _extract_tier3_cpp(path: Path, content: str) -> list[dict]:
    """Tier 3: C++ constexpr, const std::string, #define."""
    results: list[dict] = []
    seen: set[tuple[str, int]] = set()

    for m in _RE_CPP_CONSTEXPR.finditer(content):
        val = m.group(2).strip()
        if not _is_topic_like(val):
            continue
        topic = _normalize_topic(val)
        if not topic:
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Constant",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "const_name": m.group(1),
            "detected_qos": qos,
        })

    for m in _RE_CPP_CONST_STRING.finditer(content):
        val = m.group(2).strip()
        if not _is_topic_like(val):
            continue
        topic = _normalize_topic(val)
        if not topic:
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Constant",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "const_name": m.group(1),
            "detected_qos": qos,
        })

    for m in _RE_CPP_DEFINE.finditer(content):
        val = m.group(2).strip()
        if not _is_topic_like(val):
            continue
        topic = _normalize_topic(val)
        if not topic:
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Constant",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "const_name": m.group(1),
            "detected_qos": qos,
        })

    return results


def _extract_tier3_py(path: Path, content: str) -> list[dict]:
    """Tier 3: Python variable assignment (TOPIC = "/topic")."""
    results: list[dict] = []
    seen: set[tuple[str, int]] = set()

    for m in _RE_PY_CONST.finditer(content):
        val = m.group(2).strip()
        if not _is_topic_like(val):
            continue
        topic = _normalize_topic(val)
        if not topic:
            continue
        name = m.group(1)
        name_lower = name.lower()
        if not (
            val.startswith("/")
            or any(x in name_lower for x in ("topic", "graph", "map", "filter", "info"))
        ):
            continue
        key = (topic, m.start())
        if key in seen:
            continue
        seen.add(key)
        qos = _extract_qos_from_content(content, m.start())
        results.append({
            "topic": topic,
            "source_type": "Constant",
            "file_path": str(path),
            "line_number": _get_line_number(content, m.start()),
            "const_name": name,
            "detected_qos": qos,
        })

    return results


def extract_topics(package_path: Path) -> dict[str, dict]:
    """
    Extract potential topic list with 3-stage patterns.

    Returns
    -------
    dict
        {
            "/topic_name": {
                "source_type": "Literal" | "Parameter" | "Constant",
                "file_path": "src/node.cpp",
                "line_number": 42,
                "detected_qos": { "reliability": "...", "durability": "..." },
                "entity_type": "pub" | "sub" (Literal only)
            }
        }
    """
    package_path = Path(package_path).resolve()
    if not package_path.is_dir():
        return {}

    out: dict[str, dict] = {}
    # Prioritize Tier 1, then Tier 2/3 (don't overwrite - Literal is most reliable)
    all_raw: list[dict] = []

    for src_path in code_scanner.find_source_files(package_path):
        try:
            content = src_path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue

        path_str = str(src_path)

        if src_path.suffix in (".cpp", ".hpp", ".h"):
            all_raw.extend(_extract_tier1_cpp(src_path, content))
            all_raw.extend(_extract_tier2_cpp(src_path, content))
            all_raw.extend(_extract_tier3_cpp(src_path, content))
        elif src_path.suffix == ".py":
            all_raw.extend(_extract_tier1_py(src_path, content))
            all_raw.extend(_extract_tier2_py(src_path, content))
            all_raw.extend(_extract_tier3_py(src_path, content))

    # Same topic: Literal > Parameter > Constant
    priority = {"Literal": 0, "Parameter": 1, "Constant": 2}

    for r in all_raw:
        topic = r["topic"]
        if not topic:
            continue
        p_new = priority.get(r["source_type"], 99)
        existing = out.get(topic, {})
        p_old = priority.get(existing.get("source_type", ""), 99)
        if topic not in out or p_new < p_old:
            entry: dict = {
                "source_type": r["source_type"],
                "file_path": r["file_path"],
                "line_number": r["line_number"],
                "detected_qos": r.get("detected_qos") or {},
            }
            if "entity_type" in r:
                entry["entity_type"] = r["entity_type"]
            if "param_name" in r:
                entry["param_name"] = r["param_name"]
            if "const_name" in r:
                entry["const_name"] = r["const_name"]
            out[topic] = entry

    return out
