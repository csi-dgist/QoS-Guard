#!/usr/bin/env python3
"""
ROS 2 source code QoS static analyzer.

3-stage generalized scanning:
1) Global Symbol Collection: collect symbols that return/generate rclcpp::QoS
2) Internal Logic Parsing: parse method chaining like .reliable(), .keep_last(N) within definitions
3) Caller Mapping: lookup in dictionary and assign for create_publisher(..., MyQoS())
"""
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from . import qos_profile_resolver
from . import yaml_qos_loader


# Skip test/demo/doc sources. Their publishers are not deployment endpoints.
_SKIP_DIRS = {
    "test", "tests", "example", "examples", "benchmark", "benchmarks", "doc", "docs",
}


def _is_test_path(p: Path, root: Path) -> bool:
    """True if p is a test/example/doc source or a test-named file under root."""
    try:
        parts = p.relative_to(root).parts
    except ValueError:
        parts = p.parts
    if not parts:
        return False
    *dirs, name = parts
    if any(d.lower() in _SKIP_DIRS for d in dirs):
        return True
    stem = name.rsplit(".", 1)[0].lower()
    return stem.startswith("test_") or stem.endswith(("_test", "_tests"))


def find_source_files(package_path: Path) -> list[Path]:
    """Recursively find .cpp, .hpp, .h, .py source files under package path.

    Test, example, benchmark, and doc sources are excluded so they are not
    counted as deployment endpoints.
    """
    package_path = Path(package_path).resolve()
    if not package_path.is_dir():
        return []

    exts = ("*.cpp", "*.hpp", "*.h", "*.py")
    result: list[Path] = []
    for ext in exts:
        result.extend(package_path.rglob(ext))
    return sorted(
        p for p in result if p.is_file() and not _is_test_path(p, package_path)
    )


# ────────── C++ (rclcpp) patterns ──────────
# Match up to the call's '('. Args are then split by balanced-paren scan so inline chained QoS is not truncated.
_RE_CPP_CREATE_PUB_ANCHOR = re.compile(r"create_publisher\s*<[^>]+>\s*\(", re.I)
_RE_CPP_CREATE_SUB_ANCHOR = re.compile(r"create_subscription\s*<[^>]+>\s*\(", re.I)


def _split_call_args(content: str, paren_idx: int) -> list[str]:
    """Split a call's top-level, comma-separated arguments.

    paren_idx points at the opening '(' of the call. Nested (), [], {} and string
    literals are skipped so commas inside them do not split arguments. Returns the
    argument substrings (order preserved), or [] if the parenthesis never closes.
    """
    args: list[str] = []
    depth = 0
    n = len(content)
    cur = paren_idx + 1
    i = paren_idx
    while i < n:
        c = content[i]
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                args.append(content[cur:i])
                return args
        elif c in "\"'":
            q = c
            i += 1
            while i < n and content[i] != q:
                if content[i] == "\\":
                    i += 1
                i += 1
        elif c == "," and depth == 1:
            args.append(content[cur:i])
            cur = i + 1
        i += 1
    return args

# topic: "/cmd_vel" or "\"/cmd_vel\"" etc
_RE_STRING_LITERAL = re.compile(r'["\']([^"\']*)["\']')

# Parameter-variable connection: var = declare_parameter("topic_name", "map") → var → "map"
_RE_PARAM_ASSIGN_STD = re.compile(
    r"(\w+)\s*=\s*[^;]*declare_(?:or_get_)?parameter\s*"
    r"(?:<[^>]+>\s*)?"
    r"\(\s*[\"']([^\"']+)[\"']\s*,\s*std::string\s*[({\s]*[\"']([^\"']*)[\"']\s*[)}\s]*\s*\)",
    re.I | re.S,
)
_RE_PARAM_ASSIGN_QUOTE = re.compile(
    r"(\w+)\s*=\s*[^;]*declare_(?:or_get_)?parameter\s*"
    r"(?:<[^>]+>\s*)?"
    r"\(\s*[\"']([^\"']+)[\"']\s*,\s*[\"']([^\"']*)[\"']\s*\)",
    re.I | re.S,
)
# Dynamic parameter: map_topic_ = declare_or_get_parameter(name_ + "." + "map_topic", std::string("map"))
# If first argument has topic-related suffix, use default as topic
_RE_PARAM_ASSIGN_DYNAMIC = re.compile(
    r"(\w+)\s*=\s*[^;]*declare_(?:or_get_)?parameter\s*"
    r"\(\s*([^)]+),\s*std::string\s*[({\s]*[\"']([^\"']*)[\"']",
    re.I | re.S,
)
_TOPIC_PARAM_SUFFIXES = ("topic", "topic_name", "_topic", "map_topic", "graph", "info")

# QoS(10) or QoS(KeepLast(10))
_RE_QOS_CTOR = re.compile(
    r"(?:rclcpp::)?QoS\s*\(\s*(?:rclcpp::)?(?:KeepLast|KeepAll)\s*\(\s*(\d+)\s*\)\s*\)",
    re.I,
)
# rclcpp::QoS(10) - capture [^)]+ in create_publisher, ")" may be cut off like "QoS(10"
_RE_QOS_CTOR_SIMPLE = re.compile(
    r"(?:rclcpp::)?QoS\s*\(\s*(\d+)\s*\)?",
    re.I,
)

# qos.reliability(ReliabilityPolicy::Reliable)
_RE_RELIABILITY = re.compile(
    r"\.reliability\s*\(\s*(?:rclcpp::)?ReliabilityPolicy\s*::\s*(\w+)\s*\)",
    re.I,
)
_RE_DURABILITY = re.compile(
    r"\.durability\s*\(\s*(?:rclcpp::)?DurabilityPolicy\s*::\s*(\w+)\s*\)",
    re.I,
)
_RE_KEEP_LAST = re.compile(r"\.keep_last\s*\(\s*(\d+)\s*\)", re.I)
_RE_KEEP_ALL = re.compile(r"\.keep_all\s*\(\s*\)", re.I)

# ────────── Python (rclpy) patterns ──────────
# create_publisher(msg_type, topic, qos_profile) - positional 3 args
_RE_PY_CREATE_PUB_POS3 = re.compile(
    r"create_publisher\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*([^)]+)\s*\)",
    re.I | re.S,
)
# create_publisher(msg_type, topic) - positional 2 args, qos default
_RE_PY_CREATE_PUB_POS2 = re.compile(
    r"create_publisher\s*\(\s*[^,]+,\s*([^,)]+)\s*\)",
    re.I | re.S,
)
# create_subscription(msg_type, topic, callback, qos_profile) - positional 4 args
_RE_PY_CREATE_SUB_POS4 = re.compile(
    r"create_subscription\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*[^,]+,\s*([^)]+)\s*\)",
    re.I | re.S,
)
# create_subscription(msg_type, topic, callback) - positional 3 args, qos default
_RE_PY_CREATE_SUB_POS3 = re.compile(
    r"create_subscription\s*\(\s*[^,]+,\s*([^,)]+)\s*,\s*[^,]+\)",
    re.I | re.S,
)
# create_publisher(topic=..., qos_profile=...) - keyword (extract topic, qos_profile)
_RE_PY_TOPIC_KW = re.compile(
    r"topic\s*=\s*[\"']([^\"']*)[\"']|topic\s*=\s*(\w+)",
    re.I,
)
_RE_PY_QOS_KW = re.compile(
    r"qos_profile\s*=\s*([^,)]+)",
    re.I | re.S,
)
# create_publisher(..., topic="x", ...) or create_publisher(..., qos_profile=q, ...)
# For extracting keyword arguments from entire function call
_RE_PY_CREATE_PUB_CALL = re.compile(
    r"create_publisher\s*\(([^)]*(?:\([^)]*\)[^)]*)*)\)",
    re.I | re.S,
)
_RE_PY_CREATE_SUB_CALL = re.compile(
    r"create_subscription\s*\(([^)]*(?:\([^)]*\)[^)]*)*)\)",
    re.I | re.S,
)

# QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=..., depth=10)
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


def _extract_string_literal(s: str) -> str | None:
    """Extract string literal from expression. '"  /cmd_vel  "' -> '/cmd_vel'."""
    m = _RE_STRING_LITERAL.search(s.strip())
    return m.group(1).strip() if m else None


def _build_param_topic_map(content: str) -> dict[str, str]:
    """
    declare_parameter/declare_or_get_parameter → var → topic default mapping in variable assignment.
    For tracking when topic_name is a variable in create_publisher(topic_name).
    """
    result: dict[str, str] = {}
    for m in _RE_PARAM_ASSIGN_STD.finditer(content):
        var_name, param_name, default_val = m.group(1), m.group(2), m.group(3)
        if not default_val or param_name.lower() in ("plugin", "type", "class"):
            continue
        if not any(s in param_name.lower() for s in _TOPIC_PARAM_SUFFIXES):
            continue
        if not _is_valid_topic_value(default_val):
            continue
        result[var_name] = default_val.strip()

    for m in _RE_PARAM_ASSIGN_QUOTE.finditer(content):
        var_name, param_name, default_val = m.group(1), m.group(2), m.group(3)
        if not default_val or param_name.lower() in ("plugin", "type", "class"):
            continue
        if not any(s in param_name.lower() for s in _TOPIC_PARAM_SUFFIXES):
            continue
        if not _is_valid_topic_value(default_val):
            continue
        result[var_name] = default_val.strip()

    for m in _RE_PARAM_ASSIGN_DYNAMIC.finditer(content):
        var_name, first_arg, default_val = m.group(1), m.group(2), m.group(3)
        if not default_val:
            continue
        if not any(s in (first_arg or "").lower() for s in _TOPIC_PARAM_SUFFIXES):
            continue
        if not _is_valid_topic_value(default_val):
            continue
        result[var_name] = default_val.strip()

    return result


def _is_valid_topic_value(val: str) -> bool:
    """Filter non-topic values (png, yaml, file paths, etc.)."""
    v = (val or "").strip()
    if len(v) < 2:
        return False
    lower = v.lower()
    if lower.startswith(".") or lower.endswith((".png", ".yaml", ".pgm", ".bmp", ".xml")):
        return False
    if "/tmp" in lower or lower.startswith("tmp/"):
        return False
    invalid = ("png", "yaml", "tmp", "foo", "bar", "open_loop", "closed_loop",
               "_raw", "_updates")
    if lower in invalid:
        return False
    if v.startswith("_") and len(v) <= 5:
        return False
    return True


def _cpp_policy_to_dds(reliability: str) -> str:
    """Map rclcpp ReliabilityPolicy to DDS kind."""
    r = (reliability or "").upper()
    if "RELIABLE" in r:
        return "RELIABLE"
    return "BEST_EFFORT"


def _cpp_durability_to_dds(durability: str) -> str:
    """Map rclcpp DurabilityPolicy to DDS kind."""
    d = (durability or "").upper()
    if "TRANSIENT_LOCAL" in d or "TRANSIENTLOCAL" in d:
        return "TRANSIENT_LOCAL"
    if "TRANSIENT" in d:
        return "TRANSIENT"
    if "PERSISTENT" in d:
        return "PERSISTENT"
    return "VOLATILE"


# Chained QoS: QoS(KeepLast(N)).reliability(...).durability(...) (can be multiple lines)
_RE_CHAINED_QOS = re.compile(
    r"(?:rclcpp::)?QoS\s*\(\s*(?:rclcpp::)?(?:KeepLast|KeepAll)\s*\(\s*(\d+)\s*\)\s*\)"
    r"(?:[\s\S]*?\.reliability\s*\(\s*(?:rclcpp::)?ReliabilityPolicy\s*::\s*(\w+)\s*\))?"
    r"(?:[\s\S]*?\.durability\s*\(\s*(?:rclcpp::)?DurabilityPolicy\s*::\s*(\w+)\s*\))?",
    re.I,
)


def _parse_inline_qos(expr: str) -> dict[str, str] | None:
    """Resolve an inline QoS construction/chain at the call site, e.g.
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable().

    Reads depth, reliability, and durability from the expression via the
    definition-body parser. Returns None when no explicit policy is present
    (e.g. a bare variable name), so the caller can try other paths.
    """
    if not expr or "(" not in expr:
        return None
    out = qos_profile_resolver._parse_cpp_qos_body(expr)
    # Honour the int constructor rclcpp::QoS(N), which implies KeepLast(N), when a
    # KeepLast(N)/keep_last(N) did not already set the depth.
    if out["history_depth"] == "10" and out["history"] != "KEEP_ALL":
        mnum = re.search(r"(?:rclcpp::)?QoS\s*\(\s*(\d+)\s*\)", expr, re.I)
        if mnum:
            out["history_depth"] = mnum.group(1)
    explicit = (
        bool(out["reliability"])
        or bool(out["durability"])
        or out["history"] == "KEEP_ALL"
        or out["history_depth"] != "10"
    )
    return out if explicit else None


def _resolve_var_qos(var_name: str, content: str) -> dict[str, str] | None:
    """Resolve a locally declared/configured QoS variable from its full definition.

    Gathers the variable's assignment RHS, constructor arguments, and chained
    var.<method>() calls, then parses them together. Returns None if the
    variable carries no explicit policy.
    """
    v = re.escape(var_name)
    frags: list[str] = []
    ma = re.search(rf"\b{v}\s*=\s*([^;]+);", content, re.I | re.S)
    if ma:
        frags.append(ma.group(1))
    md = re.search(rf"(?:rclcpp::)?QoS\s+{v}\s*\(([^;]*)\)\s*;", content, re.I)
    if md:
        frags.append("QoS(" + md.group(1) + ")")
    for mc in re.finditer(rf"\b{v}\b((?:\s*\.\s*[A-Za-z_]\w*\s*\([^;{{}}]*\))+)", content, re.I):
        frags.append(mc.group(1))
    if not frags:
        return None
    return _parse_inline_qos(" ".join(frags))


def _extract_cpp_qos_from_context(
    content: str,
    qos_expr: str,
    qos_dict: dict[str, dict[str, str]] | None = None,
    package_contents: dict[Path, str] | None = None,
    yaml_params: dict[str, dict[str, str]] | None = None,
) -> dict[str, str]:
    """Extract QoS from context. Stage 3: Prioritize Custom QoS Dictionary lookup."""
    # ③ Member variables: package-wide context (headers+sources)
    search_contents = [content]
    if package_contents:
        search_contents = list(package_contents.values())

    out: dict[str, str] = {
        "reliability": "",
        "durability": "",
        "history": "KEEP_LAST",
        "history_depth": "10",
    }

    # Stage 3: Caller mapping - dictionary lookup for MyQoS(), nav2::qos::LatchedPublisherQoS(), etc.
    if qos_dict:
        resolved = qos_profile_resolver.resolve_qos_from_expression(qos_expr, qos_dict)
        if resolved:
            return resolved

    # Inline chained construction at the call site (common latched-publisher
    # idiom). Resolve it whole before the narrower single-policy patterns below.
    inline = _parse_inline_qos(qos_expr)
    if inline is not None:
        if yaml_params:
            inline = yaml_qos_loader.merge_yaml_qos_into(inline, yaml_params)
        return inline

    # Inline: QoS(10) or QoS(KeepLast(10))
    m = _RE_QOS_CTOR.search(qos_expr)
    if m:
        out["history_depth"] = m.group(1)
        out["history"] = "KEEP_LAST"
    else:
        m2 = _RE_QOS_CTOR_SIMPLE.search(qos_expr)
        if m2:
            out["history_depth"] = m2.group(1)
            out["history"] = "KEEP_LAST"

    # Extract variable name (when qos_expr is simple identifier)
    var_match = re.match(r"^\s*([a-zA-Z_]\w*)\s*$", qos_expr.strip())
    if var_match:
        var_name = var_match.group(1)
        # 1) ① Composition: var = SensorDataQoS().keep_last(10).reliable()
        if qos_dict:
            for search_content in search_contents:
                assign = re.search(
                    rf"\b{re.escape(var_name)}\s*=\s*([^;]+);",
                    search_content,
                    re.I | re.S,
                )
                if assign:
                    rhs = assign.group(1).strip()
                    chain_qos = qos_profile_resolver.parse_qos_chain_rhs(rhs, qos_dict)
                    if chain_qos:
                        return chain_qos
                    break

        # Resolve the QoS variable within this file only, so a reused name (e.g. qos) in another file does not leak in.
        var_qos = _resolve_var_qos(var_name, content)
        if var_qos is not None:
            if yaml_params:
                var_qos = yaml_qos_loader.merge_yaml_qos_into(var_qos, yaml_params)
            return var_qos

        # 2) Chained QoS: var = QoS(KeepLast(N)).reliability(...).durability(...)
        chained = _RE_CHAINED_QOS.search(content)
        if chained:
            out["history_depth"] = chained.group(1)
            out["history"] = "KEEP_LAST"
            if chained.lastindex and chained.lastindex >= 2 and chained.group(2):
                out["reliability"] = _cpp_policy_to_dds(chained.group(2))
            if chained.lastindex and chained.lastindex >= 3 and chained.group(3):
                out["durability"] = _cpp_durability_to_dds(chained.group(3))

        # 3) ③ Member variables: search var_name.reliability(...), .reliable() in same file only
        # (Avoid confusion with same variable names in different files)
        for line in content.split("\n"):
            if re.search(rf"\b{re.escape(var_name)}\s*\.reliable\s*\(", line, re.I):
                out["reliability"] = "RELIABLE"
            elif re.search(rf"\b{re.escape(var_name)}\s*\.best_effort\s*\(", line, re.I):
                out["reliability"] = "BEST_EFFORT"
            elif re.search(rf"\b{re.escape(var_name)}\s*\.reliability\s*\(", line, re.I):
                r = _RE_RELIABILITY.search(line)
                if r:
                    out["reliability"] = _cpp_policy_to_dds(r.group(1))
            if re.search(rf"\b{re.escape(var_name)}\s*\.durability\s*\(", line, re.I):
                d = _RE_DURABILITY.search(line)
                if d:
                    out["durability"] = _cpp_durability_to_dds(d.group(1))
            if re.search(rf"\b{re.escape(var_name)}\s*\.keep_last\s*\(", line, re.I):
                k = _RE_KEEP_LAST.search(line)
                if k:
                    out["history"] = "KEEP_LAST"
                    out["history_depth"] = k.group(1)
            if re.search(rf"\b{re.escape(var_name)}\s*\.keep_all\s*\(", line, re.I):
                if _RE_KEEP_ALL.search(line):
                    out["history"] = "KEEP_ALL"

        # 4) Declarations of form QoS(10) or QoS(KeepLast(N))
        decl = re.search(
            rf"(?:rclcpp::)?QoS\s+{re.escape(var_name)}\s*\(\s*(\d+)\s*\)",
            content,
            re.I,
        )
        if decl:
            out["history_depth"] = decl.group(1)
            out["history"] = "KEEP_LAST"

        decl2 = re.search(
            rf"(?:rclcpp::)?QoS\s+{re.escape(var_name)}\s*\(\s*"
            rf"(?:rclcpp::)?KeepLast\s*\(\s*(\d+)\s*\)\s*\)",
            content,
            re.I,
        )
        if         decl2:
            out["history_depth"] = decl2.group(1)
            out["history"] = "KEEP_LAST"

    # ② YAML: merge config/*.yaml parameters (supplement empty fields)
    if yaml_params:
        out = yaml_qos_loader.merge_yaml_qos_into(out, yaml_params)

    return out


def _extract_py_qos_from_profile(
    content: str, qos_expr: str, qos_dict: dict[str, dict[str, str]] | None = None
) -> dict[str, str]:
    """Extract QoS from Python QoSProfile(...) or variables. Stage 3: Prioritize Custom QoS Dictionary lookup."""
    out: dict[str, str] = {
        "reliability": "",
        "durability": "",
        "history": "KEEP_LAST",
        "history_depth": "10",
    }

    # Stage 3: Caller mapping - dictionary lookup for get_latched_qos(), my_qos, etc.
    if qos_dict and qos_expr.strip():
        resolved = qos_profile_resolver.resolve_qos_from_expression(qos_expr.strip(), qos_dict)
        if resolved:
            return resolved

    # Inline QoSProfile(...)
    m = _RE_PY_QOS_PROFILE.search(qos_expr)
    if m:
        args = m.group(1)
        r = _RE_PY_RELIABILITY.search(args)
        if r:
            out["reliability"] = _cpp_policy_to_dds(r.group(1))
        d = _RE_PY_DURABILITY.search(args)
        if d:
            out["durability"] = _cpp_durability_to_dds(d.group(1))
        dep = _RE_PY_DEPTH.search(args)
        if dep:
            out["history_depth"] = dep.group(1)
        return out

    # If variable name: search var = QoSProfile(...) in content
    var_match = re.match(r"^\s*([a-zA-Z_]\w*)\s*$", qos_expr.strip())
    if var_match:
        var_name = var_match.group(1)
        decl = re.search(
            rf"\b{re.escape(var_name)}\s*=\s*QoSProfile\s*\(([^)]*)\)",
            content,
            re.I | re.S,
        )
        if decl:
            args = decl.group(1)
            r = _RE_PY_RELIABILITY.search(args)
            if r:
                out["reliability"] = _cpp_policy_to_dds(r.group(1))
            d = _RE_PY_DURABILITY.search(args)
            if d:
                out["durability"] = _cpp_durability_to_dds(d.group(1))
            dep = _RE_PY_DEPTH.search(args)
            if dep:
                out["history_depth"] = dep.group(1)

    return out


@dataclass
class CodeEntity:
    """Pub/Sub entities extracted from source code."""

    source_path: Path
    entity_type: Literal["pub", "sub"]
    topic_name: str
    qos_from_code: dict[str, str]
    line_content: str


def _build_package_param_topic_map(package_contents: dict[Path, str]) -> dict[str, str]:
    """Build var → topic mapping only once per package (performance optimization)."""
    combined: dict[str, str] = {}
    for content in package_contents.values():
        combined.update(_build_param_topic_map(content))
    return combined


def _resolve_topic_from_param(
    topic_str: str,
    param_topic_map: dict[str, str] | None = None,
) -> str | None:
    """Parameter-variable connection: create_publisher(topic_name) where topic_name → declare_parameter default."""
    s = topic_str.strip()
    # Composed topic (e.g. topic_name + "_updates"): build a label from the
    # concatenated parts so distinct variants are each counted.
    if "+" in s:
        label_parts: list[str] = []
        for part in s.split("+"):
            part = part.strip()
            lit = _extract_string_literal(part)
            if lit is not None:
                label_parts.append(lit.strip())
                continue
            vm = re.match(r"^([a-zA-Z_]\w*)$", part)
            if vm:
                nm = vm.group(1)
                label_parts.append((param_topic_map or {}).get(nm) or nm)
                continue
            return None
        composed = "".join(label_parts).strip()
        return composed or None
    topic = _extract_string_literal(s)
    if topic and _is_valid_topic_value(topic):
        return topic
    var_match = re.match(r"^\s*([a-zA-Z_]\w*)\s*$", s)
    if not var_match:
        return None
    name = var_match.group(1)
    if param_topic_map and name in param_topic_map:
        return param_topic_map[name]
    # Fallback: keep the endpoint under the variable/parameter name so its QoS is
    # still validated even when the topic is not a resolvable literal.
    return name


def scan_cpp_file(
    path: Path,
    content: str,
    qos_dict: dict[str, dict[str, str]] | None = None,
    package_contents: dict[Path, str] | None = None,
    yaml_params: dict[str, dict[str, str]] | None = None,
    param_topic_map: dict[str, str] | None = None,
) -> list[CodeEntity]:
    """Extract create_publisher, create_subscription from C++ files."""
    entities: list[CodeEntity] = []

    for m in _RE_CPP_CREATE_PUB_ANCHOR.finditer(content):
        args = _split_call_args(content, m.end() - 1)
        if len(args) < 2:
            continue
        topic = _resolve_topic_from_param(args[0].strip(), param_topic_map)
        if not topic:
            continue
        qos = _extract_cpp_qos_from_context(
            content, args[1].strip(), qos_dict, package_contents, yaml_params
        )
        entities.append(
            CodeEntity(
                source_path=path,
                entity_type="pub",
                topic_name=topic,
                qos_from_code=qos,
                line_content=content[m.start():m.start() + 80],
            )
        )

    for m in _RE_CPP_CREATE_SUB_ANCHOR.finditer(content):
        args = _split_call_args(content, m.end() - 1)
        if len(args) < 2:
            continue
        topic = _resolve_topic_from_param(args[0].strip(), param_topic_map)
        if not topic:
            continue
        qos = _extract_cpp_qos_from_context(
            content, args[1].strip(), qos_dict, package_contents, yaml_params
        )
        entities.append(
            CodeEntity(
                source_path=path,
                entity_type="sub",
                topic_name=topic,
                qos_from_code=qos,
                line_content=content[m.start():m.start() + 80],
            )
        )

    return entities


def _extract_py_call_args(content: str, create_name: str) -> list[tuple[str | None, str | None]]:
    """
    Extract (topic, qos_expr) from create_publisher or create_subscription calls.

    Supports positional and keyword arguments.
    """
    results: list[tuple[str | None, str | None]] = []
    if create_name == "create_publisher":
        with_qos_re = _RE_PY_CREATE_PUB_POS3
        no_qos_re = _RE_PY_CREATE_PUB_POS2
        call_re = _RE_PY_CREATE_PUB_CALL
    else:
        with_qos_re = _RE_PY_CREATE_SUB_POS4
        no_qos_re = _RE_PY_CREATE_SUB_POS3
        call_re = _RE_PY_CREATE_SUB_CALL

    for m in with_qos_re.finditer(content):
        topic_str = m.group(1).strip()
        qos_expr = m.group(2).strip() if m.lastindex and m.lastindex >= 2 else ""
        topic = _extract_string_literal(topic_str)
        results.append((topic, qos_expr))

    for m in no_qos_re.finditer(content):
        topic_str = m.group(1).strip()
        topic = _extract_string_literal(topic_str)
        results.append((topic, None))

    for m in call_re.finditer(content):
        args = m.group(1)
        if "topic" in args.lower():
            topic_m = re.search(r"topic\s*=\s*[\"']([^\"']*)[\"']", args, re.I)
            topic = topic_m.group(1).strip() if topic_m else None
            qos_m = _RE_PY_QOS_KW.search(args)
            qos_expr = qos_m.group(1).strip() if qos_m else None
            if topic:
                results.append((topic, qos_expr))

    return results


def scan_py_file(
    path: Path, content: str, qos_dict: dict[str, dict[str, str]] | None = None
) -> list[CodeEntity]:
    """Extract create_publisher, create_subscription from Python files."""
    entities: list[CodeEntity] = []

    for topic, qos_expr in _extract_py_call_args(content, "create_publisher"):
        if topic:
            qos = _extract_py_qos_from_profile(content, qos_expr or "", qos_dict)
            entities.append(
                CodeEntity(
                    source_path=path,
                    entity_type="pub",
                    topic_name=topic,
                    qos_from_code=qos,
                    line_content="create_publisher",
                )
            )

    for topic, qos_expr in _extract_py_call_args(content, "create_subscription"):
        if topic:
            qos = _extract_py_qos_from_profile(content, qos_expr or "", qos_dict)
            entities.append(
                CodeEntity(
                    source_path=path,
                    entity_type="sub",
                    topic_name=topic,
                    qos_from_code=qos,
                    line_content="create_subscription",
                )
            )

    return entities


def scan_code(package_path: Path) -> list[CodeEntity]:
    """
    Scan package for .cpp, .hpp, .h, .py and extract QoS entities.

    Stage 1+2: Build Custom QoS Dictionary with build_qos_dictionary
    Stage 3: Dictionary lookup and mapping when calling create_publisher/subscription
    """
    package_path = Path(package_path).resolve()
    entities: list[CodeEntity] = []

    # Stage 1+2: Global symbol collection and definition parsing
    qos_dict = qos_profile_resolver.build_qos_dictionary(package_path)

    # ② YAML: config/*.yaml QoS parameters
    yaml_params = yaml_qos_loader.load_qos_from_yaml_files(package_path)

    # ③ Member variables: package-wide context (.h + .cpp)
    src_files = find_source_files(package_path)
    package_contents: dict[Path, str] = {}
    for src_path in src_files:
        try:
            package_contents[src_path] = src_path.read_text(
                encoding="utf-8", errors="ignore"
            )
        except OSError:
            pass

    param_topic_map = _build_package_param_topic_map(package_contents)

    for src_path in src_files:
        content = package_contents.get(src_path, "")
        if not content:
            continue

        if src_path.suffix in (".cpp", ".hpp", ".h"):
            entities.extend(
                scan_cpp_file(
                    src_path, content, qos_dict, package_contents, yaml_params,
                    param_topic_map=param_topic_map,
                )
            )
        elif src_path.suffix == ".py":
            entities.extend(scan_py_file(src_path, content, qos_dict))

    return entities
