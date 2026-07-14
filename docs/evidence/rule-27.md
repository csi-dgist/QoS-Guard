# Subscriber requires source-timestamp ordering the publisher does not offer

<p class="rule-ref-line">Rule 27 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. The offered ordering is weaker than requested. Fast DDS does not support source-timestamp ordering at all.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>Reader Destination Order = BY_SOURCE_TIMESTAMP</b> together with <b>Writer Destination Order = BY_RECEPTION_TIMESTAMP (weaker)</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/destination-order/">Destination Order</a> and <a href="../../qos/destination-order/">Destination Order</a>
- What QoS Guard checks: `[Writer.DESTORD < Reader.DESTORD]`

## Example

A reader that requests source-timestamp ordering will not match a reception-timestamp writer.

## How to fix it

Offer BY_SOURCE_TIMESTAMP on the writer, or relax the reader. On Fast DDS, use reception-timestamp ordering on both sides.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.17 | DESTINATION_ORDER | The value offered is considered compatible with the value requested if and only if the inequality "offered kind >= requested kind" evaluates to 'TRUE.' For the purposes of this inequality, the values of DESTINATION_ORDER kind are considered ordered such that `BY_RECEPTION_TIMESTAMP < BY_SOURCE_TIMESTAMP`. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Cyclone DDS checks destination-order compatibility during endpoint matching. A reader requesting `BY_SOURCE_TIMESTAMP` is rejected when the writer only offers `BY_RECEPTION_TIMESTAMP`.

!!! note "Cyclone DDS implementation evidence"
    ```c
    // q_qosmatch.c: reader destination order must not exceed writer destination order.
    if ((mask & QP_DESTINATION_ORDER) &&
            rd_qos->destination_order.kind > wr_qos->destination_order.kind)
    {
        *reason = DDS_DESTINATIONORDER_QOS_POLICY_ID;
        return false;
    }
    ```

Fast DDS does not support `BY_SOURCE_TIMESTAMP`, so combinations containing that value are rejected at creation time rather than evaluated as a destination-order RxO match.

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-27/rule-27-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `writer_destorder`, `reader_destorder` |
| Tested values | `writer_destorder ∈ {BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP}`, `reader_destorder ∈ {BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP}` |
| Standard-compatible cases | `writer_destorder >= reader_destorder`, including `BY_RECEPTION_TIMESTAMP ↔ BY_RECEPTION_TIMESTAMP`, `BY_SOURCE_TIMESTAMP ↔ BY_RECEPTION_TIMESTAMP`, `BY_SOURCE_TIMESTAMP ↔ BY_SOURCE_TIMESTAMP` |
| Standard-incompatible cases | `writer = BY_RECEPTION_TIMESTAMP`, `reader = BY_SOURCE_TIMESTAMP` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `writer_destorder`, `reader_destorder ∈ {BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP}` | `BY_RECEPTION_TIMESTAMP ↔ BY_RECEPTION_TIMESTAMP` matched; combinations containing `BY_SOURCE_TIMESTAMP` were rejected at creation time |
| Fast DDS 2.14.6 (Jazzy) | `writer_destorder`, `reader_destorder ∈ {BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP}` | `BY_RECEPTION_TIMESTAMP ↔ BY_RECEPTION_TIMESTAMP` matched; combinations containing `BY_SOURCE_TIMESTAMP` were rejected at creation time |
| Cyclone DDS 0.10.5 | `writer_destorder`, `reader_destorder ∈ {BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP}` | Matched when offered destination order satisfied requested destination order; match rejected when `writer = BY_RECEPTION_TIMESTAMP`, `reader = BY_SOURCE_TIMESTAMP` |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as a structural conflict:

```text
[STRUCTURAL] [cmd_vel_pub] Writer.DESTORD <-> Reader.DESTORD (Writer<Reader)
```

At runtime the two endpoints never connect. DDS matches a publisher and subscriber only when the offered QoS satisfies the requested QoS (the Requested-Offered, or RxO, rule), and here the reader's requested BY_SOURCE_TIMESTAMP is stronger than the writer's offered BY_RECEPTION_TIMESTAMP, so discovery refuses the match. No connection forms and no samples flow, and on Fast DDS the request is instead rejected at creation because source-timestamp ordering is unsupported.
