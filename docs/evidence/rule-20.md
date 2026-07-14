# Partitions filter which retained samples a late joiner replays

<p class="rule-ref-line">Rule 20 &middot; applies to publishers and subscribers &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes effort. Retained history is replayed only within the matching partition, so a late joiner in a different partition gets nothing.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>Durability = TRANSIENT_LOCAL or stronger</b> together with <b>Partition names are set (non-empty)</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/partition/">Partition</a> and <a href="../../qos/durability/">Durability</a>
- What QoS Guard checks: `[DURABL ≥ TRAN_LOCAL] ∧ [PART.names ≠ ∅]`

## Example

A latched topic retains data in partition A. A late reader in partition B matches nothing and receives no history.

## How to fix it

Make sure late joiners share the publisher's partition when you rely on durable replay.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.13 | PARTITION | A DataWriter and a DataReader communicate only when they have at least one partition name in common. Failure to match partitions is not considered an incompatible QoS. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows


Both engines gate endpoint matching through partition compatibility. When partitions do not match, the endpoints remain unmatched, so a `TRANSIENT_LOCAL` writer has no matched reader to replay historical samples to.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // EDP::valid_matching: partition mismatch is a matching failure.
    if (!matched) //Different partitions
    {
        logWarning(RTPS_EDP,
                "INCOMPATIBLE QOS (topic: " << rdata->topicName()
                << "): Different Partitions");
        reason.set(MatchingFailureMask::partitions);
    }
    return matched;
    ```

!!! note "Cyclone DDS implementation evidence"
    ```c
    // q_qosmatch.c: partition mismatch prevents endpoint matching.
    if ((mask & QP_PARTITION) && !partitions_match_p(rd_qos, wr_qos))
    {
        *reason = DDS_PARTITION_QOS_POLICY_ID;
        return false;
    }
    ```

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-20/rule-20-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `PARTITION`, `DURABL.kind` |
| Tested values | `PART ∈ {empty, A}`, `DURABL ∈ {VOLATILE, TRANSIENT_LOCAL}` |
| Rule-relevant case | `PART = empty` vs `PART = A` under the same `DURABL.kind` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `PART ∈ {empty, A}`, `DURABL ∈ {VOLATILE, TRANSIENT_LOCAL}` | `PART = A` matched, delivered, and recovered late-join samples; `PART = empty` did not match, deliver, or recover |
| Fast DDS 2.14.6 (Jazzy) | `PART ∈ {empty, A}`, `DURABL ∈ {VOLATILE, TRANSIENT_LOCAL}` | `PART = A` matched, delivered, and recovered late-join samples; `PART = empty` did not match, deliver, or recover |
| Cyclone DDS 0.10.5 | `PART ∈ {empty, A}`, `DURABL ∈ {VOLATILE, TRANSIENT_LOCAL}` | `PART = A` matched, delivered, and recovered late-join samples; `PART = empty` did not match, deliver, or recover |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard flags this as an operational conflict:

```text
[OPERATIONAL] [map_sub] [SUB] DURABL≥TRANSIENT_LOCAL <-> PART.names≠Ø
```

At runtime durable history is replayed only to readers that match the writer, and partition matching decides who matches. A late joiner sitting in a different partition never matches the writer, so it receives none of the retained samples even though the topic is the same. The effort of retaining transient-local history is wasted for any reader outside the writer's partition.
