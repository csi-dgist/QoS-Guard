# Deadline monitoring interrupted when partitions change

<p class="rule-ref-line">Rule 14 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. Leaving and re-joining a partition unmatches and rematches the pair, which can trip deadline-missed events unrelated to the writer's cadence.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Deadline period is finite</b> together with <b>Partition names are set (non-empty)</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/partition/">Partition</a> and <a href="../../qos/deadline/">Deadline</a>
- What QoS Guard checks: `[DEADLN.period ≠ ∞] ∧ [PART.names ≠ ∅]`

## Example

A control topic uses partitions and a 100 ms deadline. Switching partitions briefly unmatches the reader and fires a spurious deadline-missed.

## How to fix it

Account for partition changes in your deadline budget, or avoid changing partitions on deadline-monitored topics.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.13 | PARTITION | A DataWriter and a DataReader must have a partition name in common to communicate. Failure to match partitions is not considered an incompatible QoS and does not trigger an `OFFERED_INCOMPATIBLE_QOS` or `REQUESTED_INCOMPATIBLE_QOS` status. |
| §2.2.3.7 | DEADLINE | The DEADLINE policy defines the maximum expected period between updates for an instance. |

Rule 14 is derived from this interaction: if partition matching prevents the writer and reader from communicating, deadline monitoring may not be exercised for that peer even when a finite deadline period is configured.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Partition matching is checked during endpoint matching in both implementations. If the writer and reader partitions do not match, the endpoints are not matched, so deadline monitoring is not exercised for that peer.

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

For Rule 14, partition mismatch acts before deadline monitoring: it removes the communicating peer rather than producing a deadline-miss event.

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-14/rule-14-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `PARTITION`, `DEADLN.period` |
| Tested values | `PART ∈ {empty, A}`, `DEADLN.period ∈ {100 ms, 300 ms}` |
| Rule-relevant case | `PART = empty` vs `PART = A` under the same `DEADLN.period` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `PART ∈ {empty, A}`, `DEADLN.period ∈ {100 ms, 300 ms}` | `PART = A` matched and delivered; `PART = empty` did not match or deliver |
| Fast DDS 2.14.6 (Jazzy) | `PART ∈ {empty, A}`, `DEADLN.period ∈ {100 ms, 300 ms}` | `PART = A` matched and delivered; `PART = empty` did not match or deliver |
| Cyclone DDS 0.10.5 | `PART ∈ {empty, A}`, `DEADLN.period ∈ {100 ms, 300 ms}` | `PART = A` matched and delivered; `PART = empty` did not match or deliver |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard flags this as a functional conflict:

```text
[FUNCTIONAL] [audio_sub] [SUB] DEADLN.period>0 <-> PART.names≠Ø
```

At runtime a partition change unmatches and later rematches the writer and reader, and a partition mismatch is a matching failure rather than an incompatible-QoS event. While the pair is unmatched no samples arrive, so the finite deadline elapses and fires a deadline-missed notification that has nothing to do with the writer's actual cadence. The monitoring guarantee is broken by the partition transition, not by a slow writer.
