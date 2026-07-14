# Manual liveliness disrupted when partitions change

<p class="rule-ref-line">Rule 15 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. A partition change unmatches the pair, so manual liveliness assertions may not reach the reader and it can report the writer as lost.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Liveliness = MANUAL_BY_TOPIC</b> together with <b>Partition names are set (non-empty)</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/partition/">Partition</a> and <a href="../../qos/liveliness/">Liveliness</a>
- What QoS Guard checks: `[LIVENS = MANUAL] ∧ [PART.names ≠ ∅]`

## Example

A publisher asserts liveliness manually inside a partition. Moving partitions makes the subscriber briefly see the writer as not-alive.

## How to fix it

Keep partition membership stable on manually-asserted-liveliness topics, or use AUTOMATIC liveliness.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.13 | PARTITION | A DataWriter and a DataReader must have a partition name in common to communicate. Failure to match partitions is not considered an incompatible QoS and does not trigger an `OFFERED_INCOMPATIBLE_QOS` or `REQUESTED_INCOMPATIBLE_QOS` status. |
| §2.2.3.11 | LIVELINESS | The LIVELINESS policy controls the mechanism and lease duration used to determine whether a DataWriter is alive. |

Rule 15 is derived from this interaction: if partition matching prevents the writer and reader from communicating, manual liveliness assertions are not exercised for that peer because the matched endpoint relationship is not formed.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Partition matching is checked during endpoint matching in both implementations. If the writer and reader partitions do not match, the endpoints are not matched, so manual liveliness is not exercised for that peer.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // EDP.cpp: partition mismatch -> matching failure, not incompatible_qos
    // Partition mismatch does not trigger status change.

    if (!matched) // Different partitions
    {
        reason.set(MatchingFailureMask::partitions);
    }

    return matched;
    ```

    ```cpp
    // pairingReader: only valid matches are registered
    bool valid = valid_matching(&rdata, wdatait, no_match_reason, incompatible_qos);

    if (valid)
    {
        R->matched_writer_add(*wdatait);
    }
    ```

    Source: `EDP.cpp` lines 737-800 and 1061-1095.

Fast DDS compares writer and reader partitions during endpoint matching. If the partitions do not match, `EDP::valid_matching()` returns false with `MatchingFailureMask::partitions`, without setting `PARTITION_QOS_POLICY_ID` as an incompatible QoS. Because `matched_writer_add()` / `matched_reader_add()` are called only for valid matches, a partition mismatch silently prevents the matched peer from being formed.

!!! note "Cyclone DDS implementation evidence"
    ```c
    // q_qosmatch.c: partition compatibility check
    if ((mask & QP_PARTITION) && !partitions_match_p(rd_qos, wr_qos))
    {
        *reason = DDS_PARTITION_QOS_POLICY_ID;
        return false;
    }
    ```

    Source: `q_qosmatch.c` around the partition compatibility check.

For Rule 15, partition mismatch acts before liveliness monitoring: it removes the communicating peer rather than producing a liveliness event.

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-15/rule-15-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `PARTITION`, `LIVENS.kind` |
| Tested values | `PART ∈ {empty, A}`, `LIVENS.kind ∈ {AUTOMATIC, MANUAL_BY_TOPIC}` |
| Rule-relevant case | `PART = empty` vs `PART = A` under the same `LIVENS.kind` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `PART ∈ {empty, A}`, `LIVENS.kind ∈ {AUTOMATIC, MANUAL_BY_TOPIC}` | `PART = A` matched and delivered; `PART = empty` did not match or deliver |
| Fast DDS 2.14.6 (Jazzy) | `PART ∈ {empty, A}`, `LIVENS.kind ∈ {AUTOMATIC, MANUAL_BY_TOPIC}` | `PART = A` matched and delivered; `PART = empty` did not match or deliver |
| Cyclone DDS 0.10.5 | `PART ∈ {empty, A}`, `LIVENS.kind ∈ {AUTOMATIC, MANUAL_BY_TOPIC}` | `PART = A` matched and delivered; `PART = empty` did not match or deliver |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard flags this as a functional conflict:

```text
[FUNCTIONAL] [cmd_sub] [SUB] LIVENS=MANUAL_BY_TOPIC <-> PART.names≠Ø
```

At runtime a partition change unmatches the writer and reader, and a partition mismatch removes the peer rather than raising an incompatible-QoS event. While the pair is unmatched the writer's manual liveliness assertions cannot reach the reader, so the reader can report the writer as not-alive even though it is still running. The liveliness guarantee is broken by the partition transition rather than by a real writer failure.
