# Subscriber requires more durability than the publisher offers

<p class="rule-ref-line">Rule 23 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. The offered durability is weaker than requested, so the endpoints are incompatible.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>Reader Durability = TRANSIENT_LOCAL or stronger</b> together with <b>Writer Durability = VOLATILE (weaker)</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/durability/">Durability</a> and <a href="../../qos/durability/">Durability</a>
- What QoS Guard checks: `[Writer.DURABL < Reader.DURABL]`

## Example

A volatile publisher cannot satisfy a transient-local subscriber, so they never match.

## How to fix it

Raise the writer's durability to at least the reader's, or lower the reader's request.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.4 | DURABILITY | The value offered is considered compatible with the value requested if and only if the inequality "offered kind >= requested kind" evaluates to 'TRUE.' For the purposes of this inequality, the values of DURABILITY kind are considered ordered such that `VOLATILE < TRANSIENT_LOCAL < TRANSIENT < PERSISTENT`. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows


Both engines reject endpoint matching when the writer offers a lower durability kind than the reader requests.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // EDP::valid_matching: offered durability must satisfy requested durability.
    if (wdata->m_qos.m_durability.kind < rdata->m_qos.m_durability.kind)
    {
        incompatible_qos.set(fastdds::dds::DURABILITY_QOS_POLICY_ID);
    }
    ```

!!! note "Cyclone DDS implementation evidence"
    ```c
    // q_qosmatch.c: reader durability must not exceed writer durability.
    if ((mask & QP_DURABILITY) &&
            rd_qos->durability.kind > wr_qos->durability.kind)
    {
        *reason = DDS_DURABILITY_QOS_POLICY_ID;
        return false;
    }
    ```

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-23/rule-23-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `writer_durability`, `reader_durability` |
| Tested values | `writer_durability ∈ {VOLATILE, TRANSIENT_LOCAL}`, `reader_durability ∈ {VOLATILE, TRANSIENT_LOCAL}` |
| Valid / boundary cases | `writer = TRANSIENT_LOCAL`, `reader = TRANSIENT_LOCAL`; `writer = TRANSIENT_LOCAL`, `reader = VOLATILE`; `writer = VOLATILE`, `reader = VOLATILE` |
| Violating cases | `writer = VOLATILE`, `reader = TRANSIENT_LOCAL` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `writer_durability`, `reader_durability ∈ {VOLATILE, TRANSIENT_LOCAL}` | Matched when offered durability satisfied requested durability; match rejected when `writer = VOLATILE`, `reader = TRANSIENT_LOCAL` |
| Fast DDS 2.14.6 (Jazzy) | `writer_durability`, `reader_durability ∈ {VOLATILE, TRANSIENT_LOCAL}` | Matched when offered durability satisfied requested durability; match rejected when `writer = VOLATILE`, `reader = TRANSIENT_LOCAL` |
| Cyclone DDS 0.10.5 | `writer_durability`, `reader_durability ∈ {VOLATILE, TRANSIENT_LOCAL}` | Matched when offered durability satisfied requested durability; match rejected when `writer = VOLATILE`, `reader = TRANSIENT_LOCAL` |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as a structural conflict:

```text
[STRUCTURAL] [grid_pub] Writer.DURABL <-> Reader.DURABL (Writer<Reader)
```

At runtime the two endpoints never connect. DDS matches a publisher and subscriber only when the offered QoS satisfies the requested QoS (the Requested-Offered, or RxO, rule). Here the subscriber requests TRANSIENT_LOCAL while the publisher offers only VOLATILE, so the offered durability is weaker than requested and discovery refuses the match. No connection forms and no samples flow, with no error printed.
