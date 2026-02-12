#!/usr/bin/env python3
"""
CLI argument parsing module.

Supports three modes:
1. XML pair mode: pub.xml sub.xml <dds> <ros_version> [publish_period=<Nms>] [rtt=<Nms>]
2. Package mode: <package_path> <dds> <ros_version> [publish_period=<Nms>] [rtt=<Nms>]
3. XML list mode: --list <package_path>

dds: fast | cyclone | connext
ros_version: humble | jazzy | kilted
"""
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Union


DDS_CHOICES = ("fast", "cyclone", "connext")
ROS_CHOICES = ("humble", "jazzy", "kilted")

USAGE = f"""Usage:
  Package mode (default):
    qos_guard <package_path> [dds] [ros_version] [publish_period=<Nms>] [rtt=<Nms>]
    
  XML pair mode (optional):
    qos_guard --xml <pub.xml> <sub.xml> <dds> <ros_version> [publish_period=<Nms>] [rtt=<Nms>]

  XML list mode (optional):
    qos_guard --list <package_path>

  dds: {', '.join(DDS_CHOICES)} (default: fast)
  ros_version: {', '.join(ROS_CHOICES)} (default: humble)

Examples:
  qos_guard /path/to/ros2_package                    # Package mode with defaults
  qos_guard /path/to/ros2_package fast jazzy        # Package mode with specific DDS/ROS
  qos_guard --xml pub.xml sub.xml fast humble         # XML pair mode
  qos_guard --list /path/to/package                 # List XML files
"""

DEFAULT_PUBLISH_PERIOD_MS = 40
DEFAULT_RTT_MS = 50


@dataclass
class XmlPairArgs:
    """XML pair mode arguments."""

    pub_path: Path
    sub_path: Path
    dds: str
    ros_version: str
    publish_period_ms: int
    rtt_ns: int
    mode: Literal["xml_pair"] = "xml_pair"


@dataclass
class PackageArgs:
    """Package mode arguments."""

    package_path: Path
    dds: str = "fast"
    ros_version: str = "humble"
    publish_period_ms: int = DEFAULT_PUBLISH_PERIOD_MS
    rtt_ns: int = DEFAULT_RTT_MS * 1_000_000
    mode: Literal["package"] = "package"


@dataclass
class ListXmlArgs:
    """XML list mode arguments."""

    package_path: Path
    mode: Literal["list"] = "list"


CliArgs = Union[XmlPairArgs, PackageArgs]


def parse_list_args(argv: list[str]) -> ListXmlArgs | None:
    """Return ListXmlArgs for list mode, otherwise None."""
    if len(argv) >= 2 and argv[1].strip().lower() in ("--list", "-l"):
        if len(argv) < 3:
            sys.exit("[ERROR] list mode requires package_path.\n" + USAGE)
        path = Path(argv[2])
        if not path.exists():
            sys.exit(f"[ERROR] Path not found: {path}")
        return ListXmlArgs(package_path=path)
    return None


def load_text(p: Path) -> str:
    """Load text from file path."""
    if not p.exists():
        sys.exit(f"[ERROR] File not found: {p}")
    return p.read_text(encoding="utf-8", errors="ignore")


def _parse_period(arg: str) -> int:
    """Parse N from publish_period=<Nms> format."""
    if not arg.startswith("publish_period="):
        raise ValueError("must be publish_period=<Nms>")
    v = arg.split("=", 1)[1].strip().lower()
    if not v.endswith("ms") or not v[:-2].strip().isdigit():
        raise ValueError("value must look like '40ms'")
    return int(v[:-2])


def _parse_rtt(arg: str) -> int:
    """Parse N from rtt=<Nms> format and return in ns units."""
    if not arg.startswith("rtt="):
        raise ValueError("must be rtt=<Nms>")
    v = arg.split("=", 1)[1].strip().lower()
    if not v.endswith("ms") or not v[:-2].strip().isdigit():
        raise ValueError("rtt value must look like '50ms'")
    return int(v[:-2]) * 1_000_000


def _is_kv_arg(s: str) -> bool:
    """Check if format is publish_period= or rtt=."""
    return s.startswith("publish_period=") or s.startswith("rtt=")


def _parse_dds(s: str) -> str:
    v = s.strip().lower()
    if v not in DDS_CHOICES:
        sys.exit(f"[ERROR] dds must be one of {{{', '.join(DDS_CHOICES)}}}, got: {s}")
    return v


def _parse_ros_version(s: str) -> str:
    v = s.strip().lower()
    if v not in ROS_CHOICES:
        sys.exit(f"[ERROR] ros_version must be one of {{{', '.join(ROS_CHOICES)}}}, got: {s}")
    return v


def parse_args(argv: list[str]) -> CliArgs | ListXmlArgs:
    """
    Parse sys.argv and return CliArgs or ListXmlArgs.

    Default mode: Package mode (single path argument)
    - --list: XML list mode
    - --xml: XML pair mode
    """
    list_args = parse_list_args(argv)
    if list_args is not None:
        return list_args

    # Check for XML pair mode
    if len(argv) >= 2 and argv[1].strip().lower() in ("--xml", "-x"):
        if len(argv) < 6:
            sys.exit("[ERROR] XML pair mode requires pub.xml sub.xml dds ros_version.\n" + USAGE)
        
        pub_path = Path(argv[2])
        sub_path = Path(argv[3])
        if not pub_path.exists():
            sys.exit(f"[ERROR] File not found: {pub_path}")
        if not sub_path.exists():
            sys.exit(f"[ERROR] File not found: {sub_path}")
            
        dds = _parse_dds(argv[4])
        ros_version = _parse_ros_version(argv[5])

        publish_period_ms = DEFAULT_PUBLISH_PERIOD_MS
        rtt_ns = DEFAULT_RTT_MS * 1_000_000

        for arg in argv[6:]:
            if not _is_kv_arg(arg):
                sys.exit(f"[ERROR] Invalid argument: {arg}\n{USAGE}")
            try:
                if arg.startswith("publish_period="):
                    publish_period_ms = _parse_period(arg)
                else:
                    rtt_ns = _parse_rtt(arg)
            except ValueError as e:
                sys.exit(f"[ERROR] {e}")

        return XmlPairArgs(
            pub_path=pub_path,
            sub_path=sub_path,
            dds=dds,
            ros_version=ros_version,
            publish_period_ms=publish_period_ms,
            rtt_ns=rtt_ns,
        )

    # Default: Package mode
    if len(argv) < 2:
        sys.exit("[ERROR] Package mode requires package_path.\n" + USAGE)
        
    path = Path(argv[1])
    if not path.exists():
        sys.exit(f"[ERROR] Path not found: {path}")

    # Default values
    dds = "fast"
    ros_version = "humble"
    publish_period_ms = DEFAULT_PUBLISH_PERIOD_MS
    rtt_ns = DEFAULT_RTT_MS * 1_000_000

    # Parse optional arguments
    i = 2
    while i < len(argv):
        arg = argv[i].strip()
        
        if arg in DDS_CHOICES:
            dds = arg
        elif arg in ROS_CHOICES:
            ros_version = arg
        elif _is_kv_arg(arg):
            try:
                if arg.startswith("publish_period="):
                    publish_period_ms = _parse_period(arg)
                else:
                    rtt_ns = _parse_rtt(arg)
            except ValueError as e:
                sys.exit(f"[ERROR] {e}")
        else:
            sys.exit(f"[ERROR] Invalid argument: {arg}\n{USAGE}")
        i += 1

    return PackageArgs(
        package_path=path,
        dds=dds,
        ros_version=ros_version,
        publish_period_ms=publish_period_ms,
        rtt_ns=rtt_ns,
    )
