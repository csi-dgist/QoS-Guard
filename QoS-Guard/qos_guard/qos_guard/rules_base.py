#!/usr/bin/env python3
"""
Basic QoS rule checking module.

Apply single/cross rules to QoS profiles and return violation messages.
These rules are based on DDS standards and ROS 2 QoS policies.
"""
import math
import re
from dataclasses import dataclass
from typing import Callable, List, Tuple

from . import xml_parser_fastdds as xp

# xml_parser_fastdds re-export
deadline_enabled = xp.deadline_enabled
deadline_period_ns = xp.deadline_period_ns
lease_duration_ns = xp.lease_duration_ns
partition_list = xp.partition_list
parse_duration_field = xp.parse_duration_field
is_inf = xp.is_inf
DEADLINE_RE = xp.DEADLINE_RE
LEASE_RE = xp.LEASE_RE
ANNOUNCE_RE = xp.ANNOUNCE_RE
LIFESPAN_RE = xp.LIFESPAN_RE
INF_SET = xp.INF_SET
NON_VOLATILE = {"TRANSIENT_LOCAL", "TRANSIENT", "PERSISTENT"}

# RMW vendor-specific max_samples_per_instance defaults (for Table 39)
RMW_MPI_DEFAULTS = {"fast": "400", "cyclone": "-1", "connext": "400"}


@dataclass
class CheckContext:
    """Context information required for rule checking."""

    publish_period_ms: int
    rtt_ns: int
    dds: str = "fast"
    ros_version: str = "humble"


# ────────── Single profile rules ──────────
def rule_lifespan_vs_deadline(xml, _q, _ctx):
    """Check lifespan vs deadline consistency."""
    dl_m = DEADLINE_RE.search(xml)
    if not dl_m:
        return None
    lifespan_m = LIFESPAN_RE.search(xml)
    if not lifespan_m:
        return None
    lifespan_block = lifespan_m.group(0)
    sec_m = re.search(r"<\s*sec\s*>(\d+)</sec\s*>", lifespan_block, re.I)
    nsec_m = re.search(r"<\s*nanosec\s*>(\d+)</nanosec\s*>", lifespan_block, re.I)
    if not sec_m and not nsec_m:
        return None
    ls_sec = int(sec_m.group(1)) if sec_m else 0
    ls_nsec = int(nsec_m.group(1)) if nsec_m else 0
    dl_sec = parse_duration_field(dl_m.group(1))
    dl_nsec = parse_duration_field(dl_m.group(2))
    if dl_sec is None or dl_nsec is None:
        return None
    dl_ns = dl_sec * 1_000_000_000 + dl_nsec
    ls_ns = ls_sec * 1_000_000_000 + ls_nsec
    if ls_ns < dl_ns:
        return "Invalid QoS: LFSPAN.duration <-> DEADLN.period"
    return None


def rule_dest_order_vs_depth(_xml, q, _ctx):
    """Check BY_SOURCE_TIMESTAMP with depth > 1."""
    if q["dest_order"] != "BY_SOURCE_TIMESTAMP":
        return None
    if not q.get("history_depth", "").isdigit():
        return None
    depth = int(q["history_depth"])
    if depth <= 1:
        return "Invalid QoS: DESTORD=BY_SOURCE + HIST.depth≤1 <-> depth≥2"
    return None


def _parse_mpi(mpi_txt: str) -> int | None:
    """Parse max_samples_per_instance. -1/unlimited -> None, else int."""
    t = (mpi_txt or "").strip()
    if t in ("-1",):
        return None  # unlimited (Cyclone DDS)
    return int(t) if t.isdigit() else 0


def rule_history_vs_max_per_instance(_xml, q, _ctx):
    """[HIST.kind=KEEP_LAST] ∧ [HIST.depth > mpi] - Table 1."""
    hist_kind = q.get("history", "").strip().upper()
    depth_txt = q.get("history_depth", "").strip()
    mpi_txt = q.get("max_samples_per_instance", "").strip() or "0"
    depth = int(depth_txt) if depth_txt.isdigit() else 0
    mpi = _parse_mpi(mpi_txt)
    if mpi is None:
        return None  # unlimited, no check
    if hist_kind == "KEEP_LAST" and depth > mpi:
        return f"Invalid QoS: KEEP_LAST(depth={depth}) <-> max_samples_per_instance({mpi})"
    return None


def rule_autoenable_vs_volatile_reader(_xml, q, _ctx):
    """Warn autoenable=false with VOLATILE durability."""
    auto_off = q.get("autoenable", "").strip().upper() == "FALSE"
    is_volatile = q.get("durability", "").upper() == "VOLATILE"
    if auto_off and is_volatile:
        return "QoS warning: DURABL=VOLATILE <-> autoenable=FALSE"
    return None


def rule_max_samples_vs_per_instance(_xml, q, _ctx):
    """Check max_samples >= max_samples_per_instance."""
    max_s_txt = q.get("max_samples", "").strip()
    mpi_txt = q.get("max_samples_per_instance", "").strip()
    if max_s_txt == "-1" or mpi_txt == "-1":
        return None  # unlimited (Cyclone)
    if not max_s_txt.isdigit() or not mpi_txt.isdigit():
        return None
    max_s = int(max_s_txt)
    mpi = int(mpi_txt)
    if max_s < mpi:
        return f"Invalid QoS: max_samples({max_s}) <-> max_samples_per_instance({mpi})"
    return None


def rule_destorder_keepall_mpi(_xml, q, _ctx):
    """Warn BY_SOURCE_TIMESTAMP + KEEP_ALL + mpi=1."""
    if q.get("dest_order", "").upper() != "BY_SOURCE_TIMESTAMP":
        return None
    if q.get("history", "").upper() != "KEEP_ALL":
        return None
    if q.get("max_samples_per_instance", "").strip() != "1":
        return None
    return "Invalid QoS: DESTORD=BY_SOURCE + KEEP_ALL + mpi=1 <-> reorder buffer"


def rule_rdlife_autopurge_vs_durability(_xml, q, _ctx):
    """Check autopurge_disposed with durable QoS. [DURABL.kind≥TRANSIENT]∧[RDLIFE.autopurge_disposed=0]."""
    dur_kind = q.get("durability", "").strip().upper()
    auto_delay = q.get("autopurge_disposed_samples_delay", "").strip()
    # Table: DURABL.kind ≥ TRANSIENT (TRANSIENT, PERSISTENT only, excluding TRANSIENT_LOCAL)
    if dur_kind in ("TRANSIENT", "PERSISTENT") and auto_delay == "0":
        return "Invalid QoS: DURABL≥TRANSIENT <-> autopurge_disposed=0"
    return None


def _has_non_empty_partition(q: dict) -> bool:
    """Check if partition_list has at least one non-empty name."""
    part_list = q.get("partition_list", [])
    if not part_list:
        return False
    return any((p or "").strip() != "" for p in part_list)


def rule_durable_partition(_xml, q, _ctx):
    """[DURABL.kind≥transient local]∧[PART.names≠Ø] - Operational."""
    dur_kind = q.get("durability", "").strip().upper()
    if dur_kind not in NON_VOLATILE:
        return None
    if not _has_non_empty_partition(q):
        return None
    return "QoS warning: DURABL≥TRANSIENT_LOCAL <-> PART.names≠Ø"


def rule_deadline_partition(xml, q, _ctx):
    """[DEADLN.period>0]∧[PART.names≠Ø] - Functional."""
    if not deadline_enabled(xml):
        return None
    if not _has_non_empty_partition(q):
        return None
    return "QoS warning: DEADLN.period>0 <-> PART.names≠Ø"


def rule_liveliness_manual_partition(_xml, q, _ctx):
    """Warn MANUAL_BY_TOPIC with non-empty partition. [LIVENS.kind=manual by topic]∧[PART.names≠Ø]."""
    if q.get("liveliness", "").strip().upper() != "MANUAL_BY_TOPIC":
        return None
    if not _has_non_empty_partition(q):
        return None
    return "Invalid QoS: LIVENS=MANUAL_BY_TOPIC <-> PART.names≠Ø"


def rule_autodispose_with_exclusive(_xml, q, _ctx):
    """Warn autodispose with EXCLUSIVE ownership."""
    auto = q.get("autodispose", "").strip().upper()
    owner_kind = q.get("ownership", "").strip().upper()
    if auto == "TRUE" and owner_kind == "EXCLUSIVE":
        return "Invalid QoS: WDLIFE.autodispose=TRUE <-> OWNST=EXCLUSIVE"
    return None


def rule_lifespan_too_short_for_durability(xml, q, ctx):
    """Check lifespan >= RTT for durable QoS."""
    dur_kind = q.get("durability", "").strip().upper()
    lifespan_sec_match = re.search(
        r"<\s*lifespan\s*>.*?<\s*sec\s*>(\d+)</sec\s*>", xml, re.I | re.S
    )
    lifespan_nsec_match = re.search(
        r"<\s*lifespan\s*>.*?<\s*nanosec\s*>(\d+)</nanosec\s*>", xml, re.I | re.S
    )
    if not lifespan_sec_match and not lifespan_nsec_match:
        return None
    ls_sec = int(lifespan_sec_match.group(1)) if lifespan_sec_match else 0
    ls_nsec = int(lifespan_nsec_match.group(1)) if lifespan_nsec_match else 0
    lifespan_ns = ls_sec * 1_000_000_000 + ls_nsec
    rtt_ns = ctx.rtt_ns
    if dur_kind in NON_VOLATILE and lifespan_ns < rtt_ns:
        return "Invalid QoS: DURABL≥TRANSIENT_LOCAL + LFSPAN>0 <-> LFSPAN≥RTT"
    return None


def rule_exclusive_lease_infinite(xml, q, _ctx):
    """Warn EXCLUSIVE ownership with infinite lease_duration."""
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    m = LEASE_RE.search(xml)
    if not m:
        return None
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return "Invalid QoS: OWNST=EXCLUSIVE <-> LIVENS.lease=∞"
    return None


def rule_nowriter_delay_vs_infinite_lease(xml, q, _ctx):
    """Warn autopurge_nowriter with infinite lease."""
    nowriter_sec = q.get("nowriter_sec_r", "").strip()
    nowriter_nsec = q.get("nowriter_nsec_r", "").strip()
    try:
        sec = int(nowriter_sec) if nowriter_sec else 0
        nsec = int(nowriter_nsec) if nowriter_nsec else 0
    except ValueError:
        return None
    purge_ns = sec * 1_000_000_000 + nsec
    if purge_ns == 0:
        return None
    m = LEASE_RE.search(xml)
    if not m:
        return None
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return "Invalid QoS: RDLIFE.autopurge_nowriter>0 <-> LIVENS.lease=∞"
    return None


def rule_exclusive_deadline_infinite(xml, q, _ctx):
    """Warn EXCLUSIVE ownership with infinite DEADLINE."""
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    m = DEADLINE_RE.search(xml)
    if not m:
        return None
    sec_val = parse_duration_field(m.group(1))
    nsec_val = parse_duration_field(m.group(2))
    if sec_val is None or nsec_val is None:
        return "Invalid QoS: OWNST=EXCLUSIVE <-> DEADLN.period=∞"
    return None


def rule_lifespan_exceeds_per_instance(xml, q, ctx):
    """Check KEEP_ALL lifespan vs mpi×publish_period."""
    if q.get("history", "").strip().upper() != "KEEP_ALL":
        return None
    mpi_txt = q.get("max_samples_per_instance", "").strip()
    if not mpi_txt.isdigit():
        return None
    mpi = int(mpi_txt)
    pp_sec = ctx.publish_period_ms / 1000
    lifespan_m_sec = re.search(
        r"<\s*lifespan\s*>.*?<\s*sec\s*>(\d+)</sec\s*>", xml, re.I | re.S
    )
    lifespan_m_nsec = re.search(
        r"<\s*lifespan\s*>.*?<\s*nanosec\s*>(\d+)</nanosec\s*>", xml, re.I | re.S
    )
    if not lifespan_m_sec and not lifespan_m_nsec:
        return None
    ls_sec = int(lifespan_m_sec.group(1)) if lifespan_m_sec else 0
    ls_nsec = int(lifespan_m_nsec.group(1)) if lifespan_m_nsec else 0
    lifespan_sec = ls_sec + (ls_nsec / 1_000_000_000)
    allowed_sec = mpi * pp_sec
    if lifespan_sec > allowed_sec:
        return f"Invalid QoS: HIST=KEEP_ALL + LFSPAN({lifespan_sec:.2f}s) <-> mpi×PP({allowed_sec:.2f}s)"
    return None


def rule_keep_last_lifespan_overflow(xml, q, ctx):
    """Check KEEP_LAST lifespan vs depth×publish_period."""
    if q.get("history", "").strip().upper() != "KEEP_LAST":
        return None
    depth_txt = q.get("history_depth", "").strip()
    if not depth_txt.isdigit():
        return None
    depth = int(depth_txt)
    pp_sec = ctx.publish_period_ms / 1000
    lifespan_sec_match = re.search(r"<lifespan>.*?<sec>(\d+)</sec>", xml, re.I | re.S)
    lifespan_nsec_match = re.search(r"<lifespan>.*?<nanosec>(\d+)</nanosec>", xml, re.I | re.S)
    if not lifespan_sec_match and not lifespan_nsec_match:
        return None
    ls_sec = int(lifespan_sec_match.group(1)) if lifespan_sec_match else 0
    ls_nsec = int(lifespan_nsec_match.group(1)) if lifespan_nsec_match else 0
    lifespan_sec = ls_sec + ls_nsec / 1e9
    if lifespan_sec > depth * pp_sec:
        return f"Invalid QoS: HIST=KEEP_LAST(depth={depth}) + LFSPAN({lifespan_sec:.2f}s) <-> depth×PP({depth*pp_sec:.2f}s)"
    return None


# ────────── Stage 3: Table-based (Matching/Single) ──────────
def _autopurge_disposed_gt_zero(q: dict) -> bool:
    """Check if RDLIFE.autopurge_disposed_samples_delay > 0."""
    val = (q.get("autopurge_disposed_samples_delay") or "").strip()
    if not val:
        return False
    if val.upper() in INF_SET:
        return True
    try:
        return int(val) > 0
    except ValueError:
        return False


def rule_stage3_autodispose_autopurge_cross(pub_q: dict, sub_q: dict):
    """[WDLIFE.autodispose=FALSE] ∧ [RDLIFE.autopurge_disposed_samples_delay > 0] - Operational."""
    auto_off = pub_q.get("autodispose", "").strip().upper() == "FALSE"
    if not auto_off:
        return None
    if not _autopurge_disposed_gt_zero(sub_q):
        return None
    return "Invalid QoS: WDLIFE.autodispose=FALSE <-> RDLIFE.autopurge_disposed>0"


def rule_stage3_durable_best_effort(_xml, q, _ctx):
    """[DURABL.kind ≥ TRANSIENT_LOCAL] ∧ [RELIAB.kind = BEST_EFFORT] - Functional."""
    if q.get("durability", "").strip().upper() not in NON_VOLATILE:
        return None
    if q.get("reliability", "").strip().upper() != "BEST_EFFORT":
        return None
    return "Invalid QoS: DURABL≥TRANSIENT_LOCAL <-> RELIAB=BEST_EFFORT"


def rule_stage3_reliable_keep_last_depth(xml, q, ctx):
    """[RELIAB=RELIABLE] ∧ [HIST=KEEP_LAST] ∧ [depth < ⌈RTT/PP⌉+2] - Functional."""
    if q.get("reliability", "").strip().upper() != "RELIABLE":
        return None
    if q.get("history", "").strip().upper() != "KEEP_LAST":
        return None
    depth_txt = q.get("history_depth", "").strip()
    if not depth_txt.isdigit():
        return None
    depth = int(depth_txt)
    pp_sec = ctx.publish_period_ms / 1000
    rtt_sec = ctx.rtt_ns / 1_000_000_000
    required = math.ceil(rtt_sec / pp_sec) + 2
    if depth >= required:
        return None
    return f"Invalid QoS: RELIAB=RELIABLE + KEEP_LAST(depth={depth}) <-> ⌈RTT/PP⌉+2({required})"


def rule_stage3_reliable_keepall_mpi(xml, q, ctx):
    """[RELIAB=RELIABLE] ∧ [HIST=KEEP_ALL] ∧ [mpi < ⌈RTT/PP⌉+1] - Functional (Table: +1)."""
    if q.get("reliability", "").strip().upper() != "RELIABLE":
        return None
    if q.get("history", "").strip().upper() != "KEEP_ALL":
        return None
    mpi_txt = q.get("max_samples_per_instance", "").strip()
    if not mpi_txt.isdigit():
        return None
    mpi = int(mpi_txt)
    pp_sec = ctx.publish_period_ms / 1000
    rtt_sec = ctx.rtt_ns / 1_000_000_000
    required = math.ceil(rtt_sec / pp_sec) + 1
    if mpi >= required:
        return None
    return f"Invalid QoS: RELIAB=RELIABLE + KEEP_ALL + mpi({mpi}) <-> ⌈RTT/PP⌉+1({required})"


def rule_stage3_reliable_lifespan(xml, q, ctx):
    """[RELIAB=RELIABLE] ∧ [LFSPAN.duration < RTT×2] - Functional (Table 31)."""
    if q.get("reliability", "").strip().upper() != "RELIABLE":
        return None
    lifespan_sec_m = re.search(
        r"<\s*lifespan\s*>.*?<\s*sec\s*>(\d+)</sec\s*>", xml, re.I | re.S
    )
    lifespan_nsec_m = re.search(
        r"<\s*lifespan\s*>.*?<\s*nanosec\s*>(\d+)</nanosec\s*>", xml, re.I | re.S
    )
    if not lifespan_sec_m and not lifespan_nsec_m:
        return None
    ls_sec = int(lifespan_sec_m.group(1)) if lifespan_sec_m else 0
    ls_nsec = int(lifespan_nsec_m.group(1)) if lifespan_nsec_m else 0
    lifespan_ns = ls_sec * 1_000_000_000 + ls_nsec
    if lifespan_ns >= ctx.rtt_ns * 2:
        return None
    return "Invalid QoS: RELIAB=RELIABLE + LFSPAN <-> RTT×2"


def rule_stage3_exclusive_best_effort(_xml, q, _ctx):
    """[OWNST.kind = EXCLUSIVE] ∧ [RELIAB.kind = BEST_EFFORT] - Functional."""
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    if q.get("reliability", "").strip().upper() != "BEST_EFFORT":
        return None
    return "Invalid QoS: OWNST=EXCLUSIVE <-> RELIAB=BEST_EFFORT"


def rule_stage3_deadline_best_effort(xml, q, _ctx):
    """[DEADLN.period > 0] ∧ [RELIAB.kind = BEST_EFFORT] - Functional."""
    if not deadline_enabled(xml):
        return None
    if q.get("reliability", "").strip().upper() != "BEST_EFFORT":
        return None
    return "Invalid QoS: DEADLN.period>0 <-> RELIAB=BEST_EFFORT"


def rule_stage3_lease_deadline(xml, _q, ctx):
    """[DEADLN.period > 0] ∧ [LIVENS.lease_duration < DEADLN.period] - Functional."""
    dl_m = DEADLINE_RE.search(xml)
    ld_m = LEASE_RE.search(xml)
    if not dl_m or not ld_m:
        return None
    dl_sec = parse_duration_field(dl_m.group(1))
    dl_nsec = parse_duration_field(dl_m.group(2))
    ld_sec = parse_duration_field(ld_m.group(1))
    ld_nsec = parse_duration_field(ld_m.group(2))
    if dl_sec is None or dl_nsec is None or ld_sec is None or ld_nsec is None:
        return None
    dl_ns = dl_sec * 1_000_000_000 + (dl_nsec or 0)
    ld_ns = ld_sec * 1_000_000_000 + (ld_nsec or 0)
    if ld_ns >= dl_ns:
        return None
    return "Invalid QoS: LIVENS.lease_duration <-> DEADLN.period"


def rule_stage3_manual_liveliness_best_effort(_xml, q, _ctx):
    """[LIVENS.kind = MANUAL_BY_TOPIC] ∧ [RELIAB.kind = BEST_EFFORT] - Functional."""
    if q.get("liveliness", "").strip().upper() != "MANUAL_BY_TOPIC":
        return None
    if q.get("reliability", "").strip().upper() != "BEST_EFFORT":
        return None
    return "Invalid QoS: LIVENS=MANUAL_BY_TOPIC <-> RELIAB=BEST_EFFORT"


def rule_stage3_exclusive_deadline_short(xml, q, ctx):
    """[OWNST.kind = EXCLUSIVE] ∧ [DEADLN.period < 2×PP] - Operational."""
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    if not deadline_enabled(xml):
        return None
    dl_ns = deadline_period_ns(xml)
    if dl_ns is None:
        return None
    pub_ns = ctx.publish_period_ms * 1_000_000
    if dl_ns >= 2 * pub_ns:
        return None
    return f"Invalid QoS: OWNST=EXCLUSIVE + DEADLN.period <-> 2×PP({2*ctx.publish_period_ms}ms)"


def rule_stage3_exclusive_lease_short(xml, q, ctx):
    """[OWNST.kind = EXCLUSIVE] ∧ [LIVENS.lease_duration < 2×PP] - Operational."""
    if q.get("ownership", "").strip().upper() != "EXCLUSIVE":
        return None
    lease_ns = lease_duration_ns(xml)
    if lease_ns is None:
        return None
    pub_ns = ctx.publish_period_ms * 1_000_000
    if lease_ns >= 2 * pub_ns:
        return None
    return f"Invalid QoS: OWNST=EXCLUSIVE + LIVENS.lease <-> 2×PP({2*ctx.publish_period_ms}ms)"


def rule_stage3_autodispose_best_effort(_xml, q, _ctx):
    """[WDLIFE.autodispose = TRUE] ∧ [RELIAB.kind = BEST_EFFORT] - Functional."""
    if q.get("autodispose", "").strip().upper() != "TRUE":
        return None
    if q.get("reliability", "").strip().upper() != "BEST_EFFORT":
        return None
    return "Invalid QoS: WDLIFE.autodispose=TRUE <-> RELIAB=BEST_EFFORT"


def rule_durable_keepall_mpi_default(_xml, q, ctx):
    """[DURABL≥TRAN_LOCAL] ∧ [KEEP_ALL] ∧ [mpi=middleware default] - Table 39."""
    if q.get("durability", "").strip().upper() not in NON_VOLATILE:
        return None
    if q.get("history", "").strip().upper() != "KEEP_ALL":
        return None
    mpi_txt = (q.get("max_samples_per_instance") or "").strip()
    default_mpi = RMW_MPI_DEFAULTS.get(ctx.dds, RMW_MPI_DEFAULTS["fast"])
    if mpi_txt != default_mpi:
        return None
    return (
        f"Invalid QoS: DURABL≥TRANSIENT_LOCAL + KEEP_ALL + mpi={default_mpi}(default). "
        "Consider explicit mpi for durable+KEEP_ALL."
    )


def rule_stage3_deadline_durable(xml, q, _ctx):
    """[DEADLN.period > 0] ∧ [DURABL ≥ TRAN_LOCAL] - Operational (Table 40)."""
    if not deadline_enabled(xml):
        return None
    if q.get("durability", "").strip().upper() not in NON_VOLATILE:
        return None
    return "QoS warning: DEADLN.period>0 <-> DURABL≥TRANSIENT_LOCAL"


# ────────── Single entity: (rule_fn, severity, scope) scope: "pub"|"sub"|"both" ──────────
# Table: Check Pub/Sub separately according to Entity Scope
ENTITY_RULES: List[Tuple[Callable, str, str]] = [
    # 1: HIST↔RESLIM
    (rule_history_vs_max_per_instance, "Structural", "both"),
    # 2: RESLIM↔RESLIM
    (rule_max_samples_vs_per_instance, "Structural", "both"),
    # 3: LFSPAN→DEADLN
    (rule_lifespan_vs_deadline, "Functional", "sub"),
    # 4: HIST→DESTORD
    (rule_dest_order_vs_depth, "Functional", "sub"),
    # 5: RESLIM→DESTORD
    (rule_destorder_keepall_mpi, "Functional", "sub"),
    # 6: LFSPAN→DURABL
    (rule_lifespan_too_short_for_durability, "Functional", "pub"),
    # 7: HIST↔LFSPAN
    (rule_keep_last_lifespan_overflow, "Operational", "both"),
    # 8: RESLIM↔LFSPAN
    (rule_lifespan_exceeds_per_instance, "Operational", "both"),
    # 9: DEADLN→OWNST
    (rule_exclusive_deadline_infinite, "Functional", "sub"),
    # 10: LIVENS→OWNST
    (rule_exclusive_lease_infinite, "Functional", "sub"),
    # 11: LIVENS→RDLIFE
    (rule_nowriter_delay_vs_infinite_lease, "Functional", "sub"),
    # 12: RDLIFE→DURABL
    (rule_rdlife_autopurge_vs_durability, "Functional", "sub"),
    # 13: ENTFAC→DURABL
    (rule_autoenable_vs_volatile_reader, "Operational", "both"),
    # 14: PART→DURABL
    (rule_durable_partition, "Operational", "both"),
    # 15: PART→DEADLN
    (rule_deadline_partition, "Functional", "sub"),
    # 16: PART→LIVENS
    (rule_liveliness_manual_partition, "Functional", "sub"),
    # 17: OWNST→WDLIFE
    (rule_autodispose_with_exclusive, "Functional", "sub"),
    # 28: RELIAB→DURABL
    (rule_stage3_durable_best_effort, "Functional", "both"),
    # 29: HIST→RELIAB
    (rule_stage3_reliable_keep_last_depth, "Functional", "pub"),
    # 30: RESLIM→RELIAB
    (rule_stage3_reliable_keepall_mpi, "Functional", "pub"),
    # 31: LFSPAN→RELIAB
    (rule_stage3_reliable_lifespan, "Functional", "pub"),
    # 32: RELIAB→OWNST
    (rule_stage3_exclusive_best_effort, "Functional", "both"),
    # 33: RELIAB→DEADLN
    (rule_stage3_deadline_best_effort, "Functional", "sub"),
    # 34: LIVENS→DEADLN
    (rule_stage3_lease_deadline, "Functional", "sub"),
    # 35: RELIAB→LIVENS
    (rule_stage3_manual_liveliness_best_effort, "Functional", "both"),
    # 36: DEADLN→OWNST
    (rule_stage3_exclusive_deadline_short, "Operational", "sub"),
    # 37: LIVENS→OWNST
    (rule_stage3_exclusive_lease_short, "Operational", "sub"),
    # 38: RELIAB→WDLIFE
    (rule_stage3_autodispose_best_effort, "Functional", "pub"),
    # 39: HIST→DURABL (mpi=middleware default)
    (rule_durable_keepall_mpi_default, "Operational", "pub"),
    # 40: DURABL→DEADLN
    (rule_stage3_deadline_durable, "Operational", "sub"),
]


# ────────── Stage 2: Comparison between matched entity pairs (pub vs sub) ──────────
RELIABILITY_LEVEL = {"BEST_EFFORT": 0, "RELIABLE": 1}
DURABILITY_LEVEL = {
    "VOLATILE": 0,
    "TRANSIENT_LOCAL": 1,
    "TRANSIENT": 2,
    "PERSISTENT": 3,
}
LIVELINESS_PRIORITY = {
    "AUTOMATIC": 0,
    "MANUAL_BY_PARTICIPANT": 1,
    "MANUAL_BY_TOPIC": 2,
}


def rule_dest_order_compat(pub_q, sub_q):
    """Check destination_order compatibility."""
    w_kind = (pub_q.get("dest_order", "") or "BY_RECEPTION_TIMESTAMP").strip().upper()
    r_kind = (sub_q.get("dest_order", "") or "BY_RECEPTION_TIMESTAMP").strip().upper()
    if w_kind == "BY_RECEPTION_TIMESTAMP" and r_kind == "BY_SOURCE_TIMESTAMP":
        return "Invalid QoS: Writer.DESTORD <-> Reader.DESTORD (Writer<Reader)"
    return None


def rule_ownership_compat(pub_q, sub_q):
    """Check ownership compatibility."""
    r_kind = (sub_q.get("ownership", "") or "SHARED").strip().upper()
    w_kind = (pub_q.get("ownership", "") or "SHARED").strip().upper()
    if r_kind == "EXCLUSIVE" and w_kind != "EXCLUSIVE":
        return "Invalid QoS: Writer.OWNST <-> Reader.OWNST (Writer≠Reader)"
    return None


def rule_reliability_compat(pub_q, sub_q):
    """Check reliability compatibility."""
    w_kind = (pub_q.get("reliability", "") or "BEST_EFFORT").strip().upper()
    r_kind = (sub_q.get("reliability", "") or "BEST_EFFORT").strip().upper()
    w_lvl = RELIABILITY_LEVEL.get(w_kind, 0)
    r_lvl = RELIABILITY_LEVEL.get(r_kind, 0)
    if w_lvl < r_lvl:
        return "Invalid QoS: Writer.RELIAB <-> Reader.RELIAB (Writer<Reader)"
    return None


def rule_durability_compat(pub_q, sub_q):
    """Check durability compatibility."""
    w_kind = (pub_q.get("durability", "") or "VOLATILE").strip().upper()
    r_kind = (sub_q.get("durability", "") or "VOLATILE").strip().upper()
    w_lvl = DURABILITY_LEVEL.get(w_kind, 0)
    r_lvl = DURABILITY_LEVEL.get(r_kind, 0)
    if w_lvl < r_lvl:
        return "Invalid QoS: Writer.DURABL <-> Reader.DURABL (Writer<Reader)"
    return None


def rule_deadline_period_compat(pub_xml, sub_xml):
    """Check DEADLINE period compatibility."""
    w_ns = xp.deadline_period_ns(pub_xml)
    r_ns = xp.deadline_period_ns(sub_xml)
    if r_ns is None:
        return None
    if w_ns is None:
        return "Invalid QoS: Writer.DEADLN.period <-> Reader.DEADLN.period (Writer missing)"
    if r_ns != 0 and w_ns > r_ns:
        return "Invalid QoS: Writer.DEADLN.period <-> Reader.DEADLN.period (Writer>Reader)"
    return None


def rule_nowriter_autodispose_zero(pub_q, sub_q):
    """[W.autodispose=FALSE] ∧ [R.autopurge_nowriter=0] - Functional (Table 26)."""
    auto_off = pub_q.get("autodispose", "").strip().upper() == "FALSE"
    sec_txt = (sub_q.get("nowriter_sec_r") or "").strip()
    nsec_txt = (sub_q.get("nowriter_nsec_r") or "").strip()
    if not sec_txt and not nsec_txt:
        return None
    zero_delay = (sec_txt == "0" or sec_txt == "") and (nsec_txt == "0" or nsec_txt == "")
    if auto_off and zero_delay:
        return (
            "Invalid QoS: W.autodispose=FALSE <-> R.autopurge_nowriter=0. "
            "Samples may never be purged when all writers disappear."
        )
    return None


def rule_nowriter_autodispose_inf(pub_q, sub_q):
    """[W.autodispose=FALSE] ∧ [R.autopurge_nowriter=∞] - Operational (Table 27)."""
    auto_off = pub_q.get("autodispose", "").strip().upper() == "FALSE"
    sec_txt = sub_q.get("nowriter_sec_r", "")
    nsec_txt = sub_q.get("nowriter_nsec_r", "")
    if not sec_txt and not nsec_txt:
        return None
    inf_delay = is_inf(sec_txt) or is_inf(nsec_txt)
    if auto_off and inf_delay:
        return (
            "Invalid QoS: W.autodispose=FALSE <-> R.autopurge_nowriter=∞. "
            "Samples may never be purged when all writers disappear."
        )
    return None


def rule_partition_overlap(pub_xml, sub_xml):
    """Check Writer and Reader partition overlap."""
    w_parts = set(xp.partition_list(pub_xml))
    r_parts = set(xp.partition_list(sub_xml))
    if w_parts.isdisjoint(r_parts):
        return "Invalid QoS: Writer.PART.names <-> Reader.PART.names (∩=Ø)"
    return None


def rule_liveliness_incompatibility(pub_xml, sub_xml, pub_q, sub_q):
    """Check Writer liveliness kind and lease match Reader."""
    pub_kind = (pub_q.get("liveliness", "") or "AUTOMATIC").strip().upper()
    sub_kind = (sub_q.get("liveliness", "") or "AUTOMATIC").strip().upper()
    pub_lvl = LIVELINESS_PRIORITY.get(pub_kind, 0)
    sub_lvl = LIVELINESS_PRIORITY.get(sub_kind, 0)
    pub_lease = xp.lease_duration_ns(pub_xml)
    sub_lease = xp.lease_duration_ns(sub_xml)
    msgs = []
    if pub_lvl < sub_lvl:
        msgs.append("Writer.LIVENS.kind <-> Reader.LIVENS.kind (Writer<Reader)")
    if pub_lease is not None and sub_lease is not None and pub_lease > sub_lease:
        msgs.append("Writer.LIVENS.lease_duration <-> Reader.LIVENS.lease_duration (Writer>Reader)")
    if msgs:
        return "Invalid QoS: " + " / ".join(msgs)
    return None


# ────────── Matching checks (Pub↔Sub): Tables 18-27 ──────────
CROSS_RULES_XML: List[Tuple[Callable, str]] = [
    (rule_partition_overlap, "Structural"),  # 18: PART↔PART
    (rule_deadline_period_compat, "Structural"),  # 21: DEADLN↔DEADLN
]
CROSS_RULES_PROFILES: List[Tuple[Callable, str]] = [
    (rule_reliability_compat, "Structural"),  # 19: RELIAB↔RELIAB
    (rule_durability_compat, "Structural"),  # 20: DURABL↔DURABL
    (rule_ownership_compat, "Structural"),  # 23: OWNST↔OWNST
    (rule_dest_order_compat, "Structural"),  # 24: DESTORD↔DESTORD
    (rule_stage3_autodispose_autopurge_cross, "Operational"),  # 25: WDLIFE→RDLIFE(disposed)
    (rule_nowriter_autodispose_zero, "Functional"),  # 26: WDLIFE→RDLIFE(nowriter=0)
    (rule_nowriter_autodispose_inf, "Operational"),  # 27: WDLIFE→RDLIFE(nowriter=∞)
]
CROSS_RULES_FULL: List[Tuple[Callable, str]] = [
    (rule_liveliness_incompatibility, "Structural"),  # 22: LIVENS↔LIVENS
]


def run_checks_single_entity(
    xml: str, q: dict, ctx: CheckContext, side: str
) -> List[Tuple[str, str, str]]:
    """Single entity check (orphan PUB/SUB). Apply Pub/Sub separately according to Entity Scope."""
    warnings: List[Tuple[str, str, str]] = []
    side_lower = side.lower()
    for rule_fn, severity, scope in ENTITY_RULES:
        if scope != "both" and scope != side_lower:
            continue
        msg = rule_fn(xml, q, ctx)
        if msg:
            warnings.append((severity, side, msg))
    return warnings


def run_checks(
    pub_xml: str, sub_xml: str, pub_q: dict, sub_q: dict, ctx: CheckContext
) -> List[Tuple[str, str, str]]:
    """Single entity (Entity Scope) + matching (Pub↔Sub) check. Return (severity, side, msg)."""
    warnings: List[Tuple[str, str, str]] = []

    # Single entity: Check Pub/Sub separately according to Entity Scope
    for side, (xml, prof) in (("PUB", (pub_xml, pub_q)), ("SUB", (sub_xml, sub_q))):
        side_lower = side.lower()
        for rule_fn, severity, scope in ENTITY_RULES:
            if scope != "both" and scope != side_lower:
                continue
            msg = rule_fn(xml, prof, ctx)
            if msg:
                warnings.append((severity, side, msg))

    # Matching checks (Pub↔Sub)
    for rule_fn, severity in CROSS_RULES_XML:
        msg = rule_fn(pub_xml, sub_xml)
        if msg:
            warnings.append((severity, "CROSS", msg))

    for rule_fn, severity in CROSS_RULES_PROFILES:
        msg = rule_fn(pub_q, sub_q)
        if msg:
            warnings.append((severity, "CROSS", msg))

    for rule_fn, severity in CROSS_RULES_FULL:
        msg = rule_fn(pub_xml, sub_xml, pub_q, sub_q)
        if msg:
            warnings.append((severity, "CROSS", msg))

    return warnings
