#!/usr/bin/env python3
"""
QoS parameter YAML loader.

② QoS determination through YAML parameters.
Extract reliability, durability, depth from config/*.yaml, params/*.yaml, etc.
"""
import re
from pathlib import Path


def _read_yaml_safe(path: Path) -> str:
    """Read YAML file without pyyaml dependency."""
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def _parse_yaml_scalar(content: str, key: str) -> str | None:
    """
    Extract key: value from single-file YAML (simple pattern).
    Consider indentation and nested structure.
    """
    key_esc = re.escape(key)
    # key: value (simple)
    m = re.search(rf"^\s*{key_esc}\s*:\s*[\"']?([^\"'\s#]+)[\"']?\s*(?:#|$)", content, re.M | re.I)
    if m:
        return m.group(1).strip().strip('"\'')
    # key: "value"
    m = re.search(rf"^\s*{key_esc}\s*:\s*[\"']([^\"']*)[\"']", content, re.M | re.I)
    if m:
        return m.group(1).strip()
    return None


def _parse_yaml_node_params(content: str, node_name: str) -> dict[str, str]:
    """
    Extract QoS-related from node_name.ros__parameters under ros__parameters.
    """
    out: dict[str, str] = {}
    # node_name: / ros__parameters: / find blocks starting with **/
    node_esc = re.escape(node_name)
    # Simplistic: search key: value in entire content (ignore node scope)
    for key in ("reliability", "durability", "depth", "qos_depth", "history_depth"):
        v = _parse_yaml_scalar(content, key)
        if v:
            if key == "depth" or key == "qos_depth" or key == "history_depth":
                if v.isdigit():
                    out["history_depth"] = v
            elif key == "reliability":
                out["reliability"] = "RELIABLE" if "reliable" in v.lower() else "BEST_EFFORT"
            elif key == "durability":
                vupper = v.upper()
                if "TRANSIENT_LOCAL" in vupper or "TRANSIENTLOCAL" in vupper:
                    out["durability"] = "TRANSIENT_LOCAL"
                elif "TRANSIENT" in vupper:
                    out["durability"] = "TRANSIENT"
                elif "PERSISTENT" in vupper:
                    out["durability"] = "PERSISTENT"
                else:
                    out["durability"] = "VOLATILE"
    return out


def load_qos_from_yaml_files(package_path: Path) -> dict[str, dict[str, str]]:
    """
    Extract QoS parameters from config/*.yaml, params/*.yaml, etc.

    Returns:
        {"default": {"reliability": "RELIABLE", "history_depth": "10", ...}}
        Used to supplement empty fields in code.
    """
    package_path = Path(package_path).resolve()
    if not package_path.is_dir():
        return {}

    patterns = [
        "config/**/*.yaml", "config/**/*.yml",
        "params/**/*.yaml", "params/**/*.yml",
    ]
    files: list[Path] = []
    for pat in patterns:
        try:
            files.extend(package_path.glob(pat))
        except (OSError, ValueError):
            pass
    files = sorted(set(p for p in files if p.is_file()))

    default: dict[str, str] = {}
    for fp in files:
        content = _read_yaml_safe(fp)
        if not content:
            continue
        qos = _parse_yaml_node_params(content, "")
        for k, v in qos.items():
            if v and k not in default:
                default[k] = v

    return {"default": default} if default else {}


def merge_yaml_qos_into(
    qos: dict[str, str],
    yaml_params: dict[str, dict[str, str]] | None,
    param_key: str | None = None,
) -> dict[str, str]:
    """
    Merge QoS parameters read from YAML into qos extracted from code.

    Supplement with YAML default values only when fields are empty in code.
    """
    if not yaml_params:
        return qos
    out = dict(qos)
    src = yaml_params.get(param_key or "default") or yaml_params.get("default")
    if src:
        for k, v in src.items():
            if v and not (out.get(k) or "").strip():
                out[k] = v
    return out
