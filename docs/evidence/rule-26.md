# Publisher and subscriber disagree on ownership kind

<p class="rule-ref-line">Rule 26 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. Ownership kind must match exactly, either both SHARED or both EXCLUSIVE.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>a Writer Ownership kind</b> together with <b>a Reader Ownership kind that differs from it</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/ownership/">Ownership</a> and <a href="../../qos/ownership/">Ownership</a>
- What QoS Guard checks: `[Writer.OWNST ≠ Reader.OWNST]`

## Example

An EXCLUSIVE writer and a SHARED reader on the same topic are incompatible.

## How to fix it

Set the same Ownership kind on both the writer and the reader.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.9 | OWNERSHIP | The value of the OWNERSHIP kind offered must exactly match the one requested or else they are considered incompatible. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows


Both engines reject endpoint matching when writer and reader ownership kinds differ.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // EDP::valid_matching: offered and requested ownership kinds must match.
    if (wdata->m_qos.m_ownership.kind != rdata->m_qos.m_ownership.kind)
    {
        incompatible_qos.set(fastdds::dds::OWNERSHIP_QOS_POLICY_ID);
    }
    ```

!!! note "Cyclone DDS implementation evidence"
    ```c
    // q_qosmatch.c: ownership kind mismatch prevents endpoint matching.
    if ((mask & QP_OWNERSHIP) &&
            rd_qos->ownership.kind != wr_qos->ownership.kind)
    {
        *reason = DDS_OWNERSHIP_QOS_POLICY_ID;
        return false;
    }
    ```

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-26/rule-26-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `writer_ownership`, `reader_ownership` |
| Tested values | `writer_ownership ∈ {SHARED, EXCLUSIVE}`, `reader_ownership ∈ {SHARED, EXCLUSIVE}` |
| Valid / boundary cases | `writer_ownership = reader_ownership`, including `SHARED ↔ SHARED` and `EXCLUSIVE ↔ EXCLUSIVE` |
| Violating cases | `writer_ownership != reader_ownership`, including `SHARED ↔ EXCLUSIVE` and `EXCLUSIVE ↔ SHARED` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `writer_ownership`, `reader_ownership ∈ {SHARED, EXCLUSIVE}` | `SHARED ↔ SHARED` matched; `EXCLUSIVE ↔ SHARED` match rejected; `EXCLUSIVE ↔ EXCLUSIVE` and `SHARED ↔ EXCLUSIVE` were rejected at creation time |
| Fast DDS 2.14.6 (Jazzy) | `writer_ownership`, `reader_ownership ∈ {SHARED, EXCLUSIVE}` | Matched when ownership kinds were identical; match rejected when they differed |
| Cyclone DDS 0.10.5 | `writer_ownership`, `reader_ownership ∈ {SHARED, EXCLUSIVE}` | Matched when ownership kinds were identical; match rejected when they differed |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as a structural conflict:

```text
[STRUCTURAL] [sensor_pub] Writer.OWNST <-> Reader.OWNST (Writer≠Reader)
```

At runtime the two endpoints never connect. Ownership is compatible under the Requested-Offered (RxO) rule only when both sides choose the same kind. Here the reader requests EXCLUSIVE while the writer offers SHARED, so the kinds differ and discovery refuses the match. No connection forms and no samples flow, with no error printed.
