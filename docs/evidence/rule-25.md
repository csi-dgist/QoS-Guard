# Publisher's liveliness is weaker than the subscriber requires

<p class="rule-ref-line">Rule 25 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. A weaker offered liveliness kind, or a longer offered lease, is incompatible with the request.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>a Reader that requests a strong liveliness kind or short lease</b> together with <b>a Writer that offers a weaker kind or a longer lease</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/liveliness/">Liveliness</a> and <a href="../../qos/liveliness/">Liveliness</a>
- What QoS Guard checks: `[W.LIVENS < R.LIVENS] ∨ [W.lease > R.lease]`

## Example

The reader asks for MANUAL_BY_TOPIC with a 1 s lease, the writer offers AUTOMATIC with 2 s. They never match.

## How to fix it

Offer a liveliness kind at least as strong, and a lease no longer than, what the reader requests.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.11 | LIVELINESS kind | The inequality "offered kind >= requested kind" evaluates to 'TRUE.' For the purposes of this inequality, the values of LIVELINESS kind are considered ordered such that `AUTOMATIC < MANUAL_BY_PARTICIPANT < MANUAL_BY_TOPIC`. |
| §2.2.3.11 | LIVELINESS lease_duration | The inequality "offered lease_duration <= requested lease_duration" evaluates to TRUE. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows


Both engines reject endpoint matching when the writer offers a lower liveliness kind or a longer lease duration than the reader requests.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // EDP::valid_matching: liveliness kind and lease duration are both checked.
    if (wdata->m_qos.m_liveliness.lease_duration >
            rdata->m_qos.m_liveliness.lease_duration)
    {
        incompatible_qos.set(fastdds::dds::LIVELINESS_QOS_POLICY_ID);
    }
    if (wdata->m_qos.m_liveliness.kind < rdata->m_qos.m_liveliness.kind)
    {
        incompatible_qos.set(fastdds::dds::LIVELINESS_QOS_POLICY_ID);
    }
    ```

!!! note "Cyclone DDS implementation evidence"
    ```c
    // q_qosmatch.c: writer liveliness kind and lease must satisfy reader request.
    if ((mask & QP_LIVELINESS) &&
            rd_qos->liveliness.kind > wr_qos->liveliness.kind)
    {
        *reason = DDS_LIVELINESS_QOS_POLICY_ID;
        return false;
    }
    if ((mask & QP_LIVELINESS) &&
            rd_qos->liveliness.lease_duration < wr_qos->liveliness.lease_duration)
    {
        *reason = DDS_LIVELINESS_QOS_POLICY_ID;
        return false;
    }
    ```

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-25/rule-25-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `writer_liveliness`, `reader_liveliness` |
| Tested values | `writer_liveliness ∈ {AUTOMATIC, MANUAL_BY_PARTICIPANT, MANUAL_BY_TOPIC}`, `reader_liveliness ∈ {AUTOMATIC, MANUAL_BY_PARTICIPANT, MANUAL_BY_TOPIC}` |
| Valid / boundary cases | `writer_liveliness >= reader_liveliness`, including `AUTOMATIC ≥ AUTOMATIC`, `MANUAL_BY_PARTICIPANT ≥ AUTOMATIC`, `MANUAL_BY_PARTICIPANT ≥ MANUAL_BY_PARTICIPANT`, `MANUAL_BY_TOPIC ≥ AUTOMATIC`, `MANUAL_BY_TOPIC ≥ MANUAL_BY_PARTICIPANT`, `MANUAL_BY_TOPIC ≥ MANUAL_BY_TOPIC` |
| Violating cases | `writer_liveliness < reader_liveliness`, including `AUTOMATIC < MANUAL_BY_PARTICIPANT`, `AUTOMATIC < MANUAL_BY_TOPIC`, `MANUAL_BY_PARTICIPANT < MANUAL_BY_TOPIC` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `writer_liveliness`, `reader_liveliness ∈ {AUTOMATIC, MANUAL_BY_PARTICIPANT, MANUAL_BY_TOPIC}` | Matched when `writer_liveliness >= reader_liveliness`; match rejected when `writer_liveliness < reader_liveliness` |
| Fast DDS 2.14.6 (Jazzy) | `writer_liveliness`, `reader_liveliness ∈ {AUTOMATIC, MANUAL_BY_PARTICIPANT, MANUAL_BY_TOPIC}` | Matched when `writer_liveliness >= reader_liveliness`; match rejected when `writer_liveliness < reader_liveliness` |
| Cyclone DDS 0.10.5 | `writer_liveliness`, `reader_liveliness ∈ {AUTOMATIC, MANUAL_BY_PARTICIPANT, MANUAL_BY_TOPIC}` | Matched when `writer_liveliness >= reader_liveliness`; match rejected when `writer_liveliness < reader_liveliness` |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as a structural conflict:

```text
[STRUCTURAL] [hb_pub] Writer.LIVENS.kind <-> Reader.LIVENS.kind (Writer<Reader)
```

At runtime the two endpoints never connect. DDS matches a publisher and subscriber only when the offered QoS satisfies the requested QoS (the Requested-Offered, or RxO, rule). Here the reader requests the stronger MANUAL_BY_TOPIC kind while the writer offers only AUTOMATIC, so the offered liveliness is weaker than requested and discovery refuses the match. No connection forms and no samples flow, with no error printed.
