#!/usr/bin/env python3
"""
Executable (node) inference: Classify entities by ros2 run execution unit using only package sources.

CMake: Parse add_executable, add_library, target_link_libraries
Python: Parse setup.py/setup.cfg entry_points console_scripts
"""
import configparser
import re
from pathlib import Path
from typing import Mapping


def _norm_path(pkg: Path, raw: str) -> Path | None:
    """Normalize CMake source path to absolute path relative to package."""
    if not raw or not raw.strip():
        return None
    raw = raw.strip().strip('"')
    path = (pkg / raw).resolve()
    if path.suffix and path.exists():
        return path
    for ext in (".cpp", ".cc", ".cxx", ".hpp", ".h", ".c"):
        candidate = (pkg / (raw + ext)).resolve()
        if candidate.exists():
            return candidate
    return (pkg / raw).resolve()


def _parse_cmake_sources(content: str, pkg: Path) -> tuple[dict[str, set[Path]], dict[str, set[Path]]]:
    """(add_executable, add_library) → (exec_sources, lib_sources)."""
    exec_src: dict[str, set[Path]] = {}
    lib_src: dict[str, set[Path]] = {}

    for m in re.finditer(
        r"add_executable\s*\(\s*(\w+)\s+([^)]+)\)",
        content,
        re.I | re.S,
    ):
        name = m.group(1).strip()
        args = m.group(2)
        for part in re.split(r"\s+", args.strip()):
            if part and not part.startswith("$"):
                p = _norm_path(pkg, part)
                if p:
                    exec_src.setdefault(name, set()).add(p)

    for m in re.finditer(
        r"add_library\s*\(\s*(\w+)\s+(?:SHARED|STATIC|OBJECT|MODULE)?\s*([^)]+)\)",
        content,
        re.I | re.S,
    ):
        name = m.group(1).strip()
        args = m.group(2)
        for part in re.split(r"\s+", args.strip()):
            if part and not part.startswith("$"):
                p = _norm_path(pkg, part)
                if p:
                    lib_src.setdefault(name, set()).add(p)

    return exec_src, lib_src


def _parse_cmake_links(content: str) -> dict[str, set[str]]:
    """target_link_libraries(executable lib1 lib2) → exec → set of lib names."""
    links: dict[str, set[str]] = {}
    for m in re.finditer(
        r"target_link_libraries\s*\(\s*(\w+)\s+([^)]+)\)",
        content,
        re.I | re.S,
    ):
        target = m.group(1).strip()
        deps = re.findall(r"[\w:]+", m.group(2))
        for d in deps:
            if d and d not in ("PRIVATE", "PUBLIC", "INTERFACE"):
                links.setdefault(target, set()).add(d)
    return links


def _resolve_cmake_source_to_exec(
    pkg: Path,
    exec_sources: dict[str, set[Path]],
    lib_sources: dict[str, set[Path]],
    links: dict[str, set[str]],
) -> dict[Path, str]:
    """source path → executable name. Include all sources from exec + linked libs."""
    pkg = pkg.resolve()
    result: dict[Path, str] = {}
    for exec_name, paths in exec_sources.items():
        all_paths = set(paths)
        linked = links.get(exec_name, set())
        for lib in linked:
            all_paths.update(lib_sources.get(lib, set()))
        for p in all_paths:
            try:
                resolved = p.resolve()
                if resolved.is_file():
                    result[resolved] = exec_name
            except OSError:
                pass
    return result


def _parse_python_entry_points(pkg: Path) -> dict[Path, str]:
    """setup.py/setup.cfg → entry module path → executable name."""
    result: dict[Path, str] = {}
    pkg = pkg.resolve()

    # setup.cfg
    setup_cfg = pkg / "setup.cfg"
    if setup_cfg.exists():
        try:
            cp = configparser.ConfigParser()
            cp.read(setup_cfg, encoding="utf-8")
            if "options.entry_points" in cp:
                eps = cp["options.entry_points"]
            elif "entry_points" in cp:
                eps = cp["entry_points"]
            else:
                eps = {}
            for key, val in eps.items():
                if "console_scripts" in key or key == "console_scripts":
                    for line in val.strip().split("\n"):
                        if "=" in line:
                            exec_name, mod_spec = line.split("=", 1)
                            exec_name = exec_name.strip()
                            mod_part = mod_spec.split(":")
                            if mod_part:
                                mod_path = mod_part[0].strip()
                                if mod_path:
                                    mod_path = mod_path.replace(".", "/")
                                    for p in (
                                        pkg / mod_path / "__main__.py",
                                        pkg / f"{mod_path}.py",
                                        pkg / mod_path / "__init__.py",
                                    ):
                                        if p.exists():
                                            result[p.resolve()] = exec_name
                                            break
        except Exception:
            pass

    # setup.py - regex fallback
    setup_py = pkg / "setup.py"
    if setup_py.exists():
        try:
            content = setup_py.read_text(encoding="utf-8", errors="ignore")
            for m in re.finditer(
                r"[\"']([\w_]+)[\"']\s*:\s*[\"']([\w.]+):[\w]+[\"']",
                content,
            ):
                exec_name, mod_path = m.group(1), m.group(2)
                mod_path = mod_path.replace(".", "/")
                for candidate in (
                    pkg / mod_path / "__main__.py",
                    pkg / f"{mod_path}.py",
                    pkg / mod_path / "__init__.py",
                ):
                    if candidate.exists():
                        result[candidate.resolve()] = exec_name
                        break
        except Exception:
            pass

    return result


def get_source_to_executable_map(package_path: Path) -> Mapping[Path, str]:
    """
    Package source path → executable name mapping.

    CMake: Parse add_executable, add_library, target_link_libraries
    Python: Parse setup.py/setup.cfg entry_points
    """
    pkg = Path(package_path).resolve()
    if not pkg.is_dir():
        return {}

    result: dict[Path, str] = {}

    # CMake
    cmake = pkg / "CMakeLists.txt"
    if cmake.exists():
        try:
            content = cmake.read_text(encoding="utf-8", errors="ignore")
            exec_src, lib_src = _parse_cmake_sources(content, pkg)
            links = _parse_cmake_links(content)
            cmake_map = _resolve_cmake_source_to_exec(
                pkg, exec_src, lib_src, links
            )
            for p, name in cmake_map.items():
                if p.is_file():
                    result[p] = name
        except Exception:
            pass

    # Python
    py_map = _parse_python_entry_points(pkg)
    for p, name in py_map.items():
        result[p] = name

    return result


def assign_fallback_nodes(
    entities: list,
    *,
    node_attr: str = "node_name",
    source_attr: str = "source_path",
    profile_attr: str = "profile_name",
) -> None:
    """
    Assign A, B, C... fallback to entities with empty node_name.

    Modify entities in-place. Grouping: (source_path, profile_name) basis.
    """
    needs_fallback: list[tuple[int, str]] = []
    for i, e in enumerate(entities):
        node = getattr(e, node_attr, None) or (e.get(node_attr) if isinstance(e, dict) else None)
        if not (node or "").strip():
            src = getattr(e, source_attr, None) or (e.get(source_attr) if isinstance(e, dict) else None)
            prof = getattr(e, profile_attr, None) or (e.get(profile_attr) if isinstance(e, dict) else None)
            src_str = str(src) if src else ""
            prof_str = (prof or "").strip()
            key = f"{src_str}|{prof_str}"
            needs_fallback.append((i, key))

    if not needs_fallback:
        return

    seen_keys: dict[str, str] = {}
    for i, key in needs_fallback:
        if key not in seen_keys:
            idx = len(seen_keys)
            seen_keys[key] = chr(ord("A") + idx) if idx < 26 else f"N{idx}"
        label = seen_keys[key]
        if isinstance(entities[i], dict):
            entities[i][node_attr] = label
        else:
            setattr(entities[i], node_attr, label)
