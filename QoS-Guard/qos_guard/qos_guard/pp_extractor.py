#!/usr/bin/env python3
"""
Publish-period auto-extractor.

Scans a ROS 2 package's source (C++ rclcpp, Python rclpy) for the timer / rate
constructs that drive publishers, converts every period found to milliseconds,
and reports the *smallest* one. The fastest publisher yields the highest reliable
buffer threshold (depth >= ceil(2*RTT/PP)+1), so using the minimum period is the
most conservative choice for the time-based QoS rules (31/32/33/38/39).

Recognized constructs
---------------------
C++ (rclcpp):
    create_wall_timer(<dur>, cb) / create_timer([clock,] <dur>, cb)
        <dur> = milliseconds(X) / seconds(X) / std::chrono::duration<double>(x)
                or a chrono literal (Xms, Xus, Xns, Xs, e.g. 100ms, 1s, 500ms)
    rclcpp::Rate / WallRate / GenericRate(hz)   -> period = 1000 / hz
Python (rclpy):
    node.create_timer(period_sec, cb)           -> period_sec * 1000
    node.create_rate(hz)                         -> 1000 / hz
"""
import math
import re
from pathlib import Path

# Source extensions scanned for periods.
_CPP_EXTS = (".cpp", ".hpp", ".h", ".cc", ".cxx")
_PY_EXTS = (".py",)

# Skip test/demo/doc sources. A timer/rate here does not drive the deployed publisher.
_SKIP_DIRS = {
    "test", "tests", "example", "examples",
    "doc", "docs", "benchmark", "benchmarks",
}


def _is_nondeployment(rel_parts: tuple[str, ...]) -> bool:
    """True if a package-relative path is a test/example/doc source or test-named file."""
    *dirs, name = rel_parts
    if any(d.lower() in _SKIP_DIRS for d in dirs):
        return True
    stem = name.rsplit(".", 1)[0].lower()
    return stem.startswith("test_") or stem.endswith(("_test", "_tests"))

# ── C++ construct anchors (match up to and including the opening paren) ──
_CPP_TIMER_RE = re.compile(r"\bcreate_(?:wall_)?timer\s*\(", re.I)
# rclcpp::Rate / WallRate / GenericRate<...> [var](hz)
_CPP_RATE_RE = re.compile(
    r"\b(?:rclcpp\s*::\s*)?(?:Wall|Generic)?Rate\b\s*(?:<[^>]*>)?\s*"
    r"(?:[A-Za-z_]\w*\s*)?\(",
    re.I,
)

# ── Python construct anchors ──
_PY_TIMER_RE = re.compile(r"\bcreate_timer\s*\(", re.I)
_PY_RATE_RE = re.compile(r"\bcreate_rate\s*\(", re.I)

# ── Duration sub-patterns (searched inside a call's argument span) ──
_CHRONO_FACTORY_RE = re.compile(
    r"(?:std\s*::\s*chrono\s*::\s*)?"
    r"(nanoseconds|microseconds|milliseconds|seconds|minutes|hours)\s*"
    r"\(\s*([-+]?\d+(?:\.\d+)?)\s*\)",
    re.I,
)
_CHRONO_DURATION_RE = re.compile(
    r"(?:std\s*::\s*chrono\s*::\s*)?duration\s*<[^>]*>\s*"
    r"\(\s*([-+]?\d+(?:\.\d+)?)\s*\)",
    re.I,
)
# Chrono literal: 100ms, 1s, 500ms, 20us, 5ns (order ms|us|ns before s).
_CHRONO_LITERAL_RE = re.compile(r"(?<![\w.])(\d+(?:\.\d+)?)(ms|us|ns|s)\b")
_NUMBER_RE = re.compile(r"[-+]?\d+(?:\.\d+)?")

_FACTORY_TO_MS = {
    "nanoseconds": 1e-6,
    "microseconds": 1e-3,
    "milliseconds": 1.0,
    "seconds": 1000.0,
    "minutes": 60000.0,
    "hours": 3600000.0,
}
_LITERAL_TO_MS = {"ns": 1e-6, "us": 1e-3, "ms": 1.0, "s": 1000.0}


def _call_span(content: str, open_idx: int) -> str:
    """Return the argument text between the balanced parens starting at open_idx."""
    depth = 0
    start = open_idx + 1
    for i in range(open_idx, len(content)):
        c = content[i]
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return content[start:i]
    return content[start:]


def _first_arg(span: str) -> str:
    """Return the first top-level (comma-separated) argument of a call span."""
    depth = 0
    for i, c in enumerate(span):
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif c == "," and depth == 0:
            return span[:i]
    return span


def _duration_ms(expr: str) -> float | None:
    """Convert a std::chrono duration expression to milliseconds, or None."""
    m = _CHRONO_FACTORY_RE.search(expr)
    if m:
        return float(m.group(2)) * _FACTORY_TO_MS[m.group(1).lower()]
    m = _CHRONO_DURATION_RE.search(expr)
    if m:
        return float(m.group(1)) * 1000.0  # duration<double> is seconds
    m = _CHRONO_LITERAL_RE.search(expr)
    if m:
        return float(m.group(1)) * _LITERAL_TO_MS[m.group(2).lower()]
    return None


def _hz_to_ms(expr: str, content: str) -> float | None:
    """Convert a rate/frequency argument (Hz, or a resolvable var) to a period in ms."""
    expr = expr.strip()
    # A rate object may also be constructed from a chrono duration.
    dur = _duration_ms(expr)
    if dur is not None:
        return dur if dur > 0 else None
    m = re.fullmatch(r"[-+]?\d+(?:\.\d+)?", expr)
    if not m:
        val = _resolve_var_number(expr, content)
        if val is None:
            return None
        hz = val
    else:
        hz = float(expr)
    return 1000.0 / hz if hz > 0 else None


def _resolve_var_number(expr: str, content: str) -> float | None:
    """Resolve a bare identifier to a numeric literal assigned elsewhere in the file."""
    idm = re.fullmatch(r"[A-Za-z_][\w.]*", expr.strip())
    if not idm:
        return None
    name = re.escape(expr.strip().split(".")[-1])
    vm = re.search(rf"\b{name}\s*=\s*([-+]?\d+(?:\.\d+)?)", content)
    return float(vm.group(1)) if vm else None


def _seconds_arg_to_ms(expr: str, content: str) -> float | None:
    """Convert a Python create_timer period (seconds, or resolvable var) to ms."""
    expr = expr.strip()
    if re.fullmatch(r"[-+]?\d+(?:\.\d+)?", expr):
        return float(expr) * 1000.0
    val = _resolve_var_number(expr, content)
    return val * 1000.0 if val is not None else None


def _cpp_periods(content: str) -> list[tuple[float, str]]:
    """Return (period_ms, construct_label) for every C++ timer/rate in content."""
    out: list[tuple[float, str]] = []
    for m in _CPP_TIMER_RE.finditer(content):
        span = _call_span(content, m.end() - 1)
        ms = _duration_ms(span)
        if ms is not None and ms > 0:
            out.append((ms, "create_wall_timer/create_timer"))
    for m in _CPP_RATE_RE.finditer(content):
        span = _call_span(content, m.end() - 1)
        arg = _first_arg(span)
        ms = _hz_to_ms(arg, content)
        if ms is not None and ms > 0:
            out.append((ms, "rclcpp::Rate"))
    return out


def _py_periods(content: str) -> list[tuple[float, str]]:
    """Return (period_ms, construct_label) for every rclpy timer/rate in content."""
    out: list[tuple[float, str]] = []
    for m in _PY_TIMER_RE.finditer(content):
        span = _call_span(content, m.end() - 1)
        kw = re.search(r"timer_period_sec\s*=\s*([-+]?\d+(?:\.\d+)?)", span, re.I)
        if kw:
            ms = float(kw.group(1)) * 1000.0
        else:
            ms = _seconds_arg_to_ms(_first_arg(span), content)
        if ms is not None and ms > 0:
            out.append((ms, "create_timer"))
    for m in _PY_RATE_RE.finditer(content):
        span = _call_span(content, m.end() - 1)
        ms = _hz_to_ms(_first_arg(span), content)
        if ms is not None and ms > 0:
            out.append((ms, "create_rate"))
    return out


def periods_in_source(path: Path, content: str) -> list[tuple[float, str]]:
    """Dispatch by file suffix and return all (period_ms, label) found in content."""
    suffix = path.suffix.lower()
    if suffix in _CPP_EXTS:
        return _cpp_periods(content)
    if suffix in _PY_EXTS:
        return _py_periods(content)
    return []


def _to_period_int(ms: float) -> int:
    """Convert a fractional millisecond period to a conservative integer (floor, min 1)."""
    return max(1, math.floor(ms + 1e-9))


def extract_min_publish_period_ms(
    package_path: Path,
) -> tuple[int | None, str | None]:
    """
    Scan the package for timer/rate periods and return the smallest (in ms).

    Returns
    -------
    (period_ms, source) where source is "<file> [<construct>]" of the fastest
    publisher found, or (None, None) when the package declares no timer/rate.
    """
    package_path = Path(package_path).resolve()
    if not package_path.is_dir():
        return None, None

    exts = _CPP_EXTS + _PY_EXTS
    best_ms: float | None = None
    best_src: str | None = None
    for src in sorted(package_path.rglob("*")):
        if not src.is_file() or src.suffix.lower() not in exts:
            continue
        if _is_nondeployment(src.relative_to(package_path).parts):
            continue
        try:
            content = src.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for ms, label in periods_in_source(src, content):
            if best_ms is None or ms < best_ms:
                best_ms = ms
                best_src = f"{src.name} [{label}]"

    if best_ms is None:
        return None, None
    return _to_period_int(best_ms), best_src


# ── Per-topic publish-period association ──────────────────────────────────
# Each publisher's period comes from the timer whose callback publishes it, so
# topics in one node can differ. Topics with no timer are omitted (caller
# applies the default).
_STRING_LIT_RE = re.compile(r"""["']([^"']*)["']""")
_CPP_PUB_ASSIGN_RE = re.compile(
    r"([A-Za-z_]\w*)\s*=\s*[^;=]*?create_publisher\s*<[^>]+>\s*\(", re.I
)
_PY_PUB_ASSIGN_RE = re.compile(
    r"([A-Za-z_][\w.]*)\s*=\s*[^\n=]*?create_publisher\s*\(", re.I
)
_PUBLISH_CALL = r"(?:->|\.)\s*publish\s*\("


def _matching_brace(content: str, open_idx: int) -> int | None:
    """Index of the '}' matching the '{' at open_idx, or None."""
    depth = 0
    for i in range(open_idx, len(content)):
        c = content[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i
    return None


def _nth_top_arg(span: str, n: int) -> str | None:
    """Return the n-th (0-based) top-level comma-separated argument of a span."""
    depth = 0
    start = 0
    idx = 0
    for i, c in enumerate(span):
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif c == "," and depth == 0:
            if idx == n:
                return span[start:i]
            idx += 1
            start = i + 1
    return span[start:] if idx == n else None


def _cpp_publisher_topics(content: str) -> dict[str, str]:
    """{publisher_var: topic} for C++ create_publisher assignments."""
    out: dict[str, str] = {}
    for m in _CPP_PUB_ASSIGN_RE.finditer(content):
        span = _call_span(content, m.end() - 1)
        first = (_first_arg(span) or "").strip()
        lit = _STRING_LIT_RE.search(first)
        if lit:
            out[m.group(1)] = lit.group(1).strip()
        elif re.fullmatch(r"[A-Za-z_]\w*", first):
            out[m.group(1)] = first
    return out


def _cpp_timer_bodies(content: str) -> list[tuple[float, str]]:
    """[(period_ms, body)] for C++ timers. Body spans the lambda and, for a bound
    member callback, that method's body."""
    out: list[tuple[float, str]] = []
    for m in _CPP_TIMER_RE.finditer(content):
        span = _call_span(content, m.end() - 1)
        ms = _duration_ms(span)
        if ms is None or ms <= 0:
            continue
        body = span
        bm = re.search(r"&\s*[\w:]*::(\w+)", span)
        if bm:
            defm = re.search(
                rf"\b{re.escape(bm.group(1))}\s*\([^;{{}}]*\)\s*(?:const\s*)?\{{",
                content,
            )
            if defm:
                end = _matching_brace(content, defm.end() - 1)
                if end is not None:
                    body = body + " " + content[defm.end():end]
        out.append((ms, body))
    return out


def _py_file_topic_periods(content: str) -> dict[str, float]:
    """{topic: period_ms} associating each Python publisher's topic with the
    fastest timer/rate in the same file."""
    pubs: dict[str, str] = {}
    for m in _PY_PUB_ASSIGN_RE.finditer(content):
        span = _call_span(content, m.end() - 1)
        topic_arg = _nth_top_arg(span, 1)  # create_publisher(msg_type, topic, qos)
        if topic_arg is None:
            continue
        lit = _STRING_LIT_RE.search(topic_arg)
        if lit:
            pubs[m.group(1)] = lit.group(1).strip()
    if not pubs:
        return {}
    periods = [ms for ms, _ in _py_periods(content)]
    if not periods:
        return {}
    fastest = min(periods)
    return {topic: fastest for topic in pubs.values()}


def extract_topic_periods(package_path: Path) -> dict[str, int]:
    """
    Map each publisher's topic to the publish period (ms) of the timer that
    drives it. Topics with no identifiable timer are omitted, so the caller falls
    back to the conservative default for them.
    """
    package_path = Path(package_path).resolve()
    result: dict[str, int] = {}
    if not package_path.is_dir():
        return result
    exts = _CPP_EXTS + _PY_EXTS
    for src in sorted(package_path.rglob("*")):
        if not src.is_file() or src.suffix.lower() not in exts:
            continue
        if _is_nondeployment(src.relative_to(package_path).parts):
            continue
        try:
            content = src.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if src.suffix.lower() in _CPP_EXTS:
            var_topic = _cpp_publisher_topics(content)
            if not var_topic:
                continue
            for period, body in _cpp_timer_bodies(content):
                for var, topic in var_topic.items():
                    if re.search(rf"\b{re.escape(var)}\b\s*{_PUBLISH_CALL}", body):
                        p = _to_period_int(period)
                        result[topic] = min(result.get(topic, p), p)
        else:
            for topic, period in _py_file_topic_periods(content).items():
                p = _to_period_int(period)
                result[topic] = min(result.get(topic, p), p)
    return result
