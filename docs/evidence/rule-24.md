# Publisher's deadline is slower than the subscriber demands

<p class="rule-ref-line">Rule 24 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. An offered deadline longer than requested is incompatible, so the pair never matches.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>Writer Deadline period P</b> together with <b>Reader Deadline period smaller than P</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/deadline/">Deadline</a> and <a href="../../qos/deadline/">Deadline</a>
- What QoS Guard checks: `[Writer.DEADLN.period > Reader.DEADLN.period]`

## Example

The writer promises a sample every 200 ms, but the reader demands every 100 ms. They are incompatible and do not connect.

## How to fix it

Make the writer's Deadline period at most the reader's requested period.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.7 | DEADLINE | The value offered is considered compatible with the value requested if and only if the inequality "offered deadline period <= requested deadline period" evaluates to 'TRUE.' |

<hr class="evidence-subsection-divider">

#### What the engine source code shows


Both engines reject endpoint matching when the writer offers a deadline period longer than the reader requests.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // EDP::valid_matching: offered deadline period must not exceed requested period.
    if (wdata->m_qos.m_deadline.period > rdata->m_qos.m_deadline.period)
    {
        incompatible_qos.set(fastdds::dds::DEADLINE_QOS_POLICY_ID);
    }
    ```

!!! note "Cyclone DDS implementation evidence"
    ```c
    // q_qosmatch.c: writer deadline period must not exceed reader deadline period.
    if ((mask & QP_DEADLINE) &&
            rd_qos->deadline.deadline < wr_qos->deadline.deadline)
    {
        *reason = DDS_DEADLINE_QOS_POLICY_ID;
        return false;
    }
    ```

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-24/rule-24-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `writer_deadline_ms`, `reader_deadline_ms` |
| Tested values | `writer_deadline_ms ∈ {50, 100, 200}`, `reader_deadline_ms ∈ {50, 100, 200}` |
| Valid / boundary cases | `writer_deadline_ms <= reader_deadline_ms`, including `50 ≤ 50`, `50 ≤ 100`, `50 ≤ 200`, `100 ≤ 100`, `100 ≤ 200`, `200 ≤ 200` |
| Violating cases | `writer_deadline_ms > reader_deadline_ms`, including `100 > 50`, `200 > 50`, `200 > 100` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `writer_deadline_ms`, `reader_deadline_ms ∈ {50, 100, 200}` | Matched when `writer_deadline_ms <= reader_deadline_ms`; match rejected when `writer_deadline_ms > reader_deadline_ms` |
| Fast DDS 2.14.6 (Jazzy) | `writer_deadline_ms`, `reader_deadline_ms ∈ {50, 100, 200}` | Matched when `writer_deadline_ms <= reader_deadline_ms`; match rejected when `writer_deadline_ms > reader_deadline_ms` |
| Cyclone DDS 0.10.5 | `writer_deadline_ms`, `reader_deadline_ms ∈ {50, 100, 200}` | Matched when `writer_deadline_ms <= reader_deadline_ms`; match rejected when `writer_deadline_ms > reader_deadline_ms` |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as a structural conflict:

```text
[STRUCTURAL] [pose_pub] Writer.DEADLN.period <-> Reader.DEADLN.period (Writer>Reader)
```

At runtime the two endpoints never connect. DDS matches a publisher and subscriber only when the offered QoS satisfies the requested QoS (the Requested-Offered, or RxO, rule). A deadline is compatible only when the offered period is at most the requested period, but here the writer offers a 2 s period against the reader's 1 s request, so discovery refuses the match. No connection forms and no samples flow, with no error printed.
