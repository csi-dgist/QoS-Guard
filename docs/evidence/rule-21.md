# Publisher and subscriber share no partition name

<p class="rule-ref-line">Rule 21 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. With no shared partition name the endpoints never match, even on the same topic.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>a Writer partition set</b> together with <b>a Reader partition set that does not overlap it</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/partition/">Partition</a> and <a href="../../qos/partition/">Partition</a>
- What QoS Guard checks: `[Writer.PART ∩ Reader.PART] = ∅`

## Example

Writer in partition 'left', reader in partition 'right', same topic. They never communicate.

## How to fix it

Give the writer and reader at least one common partition name, or use wildcards that overlap.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.13 | PARTITION | For a DataReader to see the changes made to an instance by a DataWriter, not only the Topic must match, but also they must share a common partition. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows


Both engines evaluate partition overlap during endpoint matching. If the writer and reader partitions do not overlap, the endpoints remain unmatched.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // EDP::valid_matching: partition mismatch is a matching failure.
    if (!matched) //Different partitions
    {
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
| Dataset | [Download CSV](../data/evidence/rule-21/rule-21-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `writer_partition`, `reader_partition` |
| Tested values | `writer_partition ∈ {empty, A, B, overlap}`, `reader_partition ∈ {empty, A, B, overlap}` |
| Valid / boundary cases | Overlapping partitions, including `A ↔ A`, `A ↔ overlap`, `B ↔ B`, `B ↔ overlap`, `overlap ↔ A`, `overlap ↔ B`, `overlap ↔ overlap`, `empty ↔ empty` |
| Violating cases | Non-overlapping partitions, including `A ↔ B`, `A ↔ empty`, `B ↔ A`, `B ↔ empty`, `empty ↔ A`, `empty ↔ B`, `empty ↔ overlap`, `overlap ↔ empty` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `writer_partition`, `reader_partition ∈ {empty, A, B, overlap}` | Matched when writer and reader partitions overlapped; match rejected when they did not overlap |
| Fast DDS 2.14.6 (Jazzy) | `writer_partition`, `reader_partition ∈ {empty, A, B, overlap}` | Matched when writer and reader partitions overlapped; match rejected when they did not overlap |
| Cyclone DDS 0.10.5 | `writer_partition`, `reader_partition ∈ {empty, A, B, overlap}` | Matched when writer and reader partitions overlapped; match rejected when they did not overlap |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as a structural conflict:

```text
[STRUCTURAL] [lidar_pub] Writer.PART.names <-> Reader.PART.names (∩=Ø)
```

At runtime the writer and reader never connect. DDS requires a publisher and subscriber to share at least one partition name before they match, even on the same topic. With disjoint partitions there is no common name, so endpoint matching fails and no samples flow, with no error printed.
