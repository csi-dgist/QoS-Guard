# Subscriber requires reliable, publisher offers best-effort

<p class="rule-ref-line">Rule 22 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. The offered reliability is weaker than requested, so the pair is incompatible and never matches.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>Reader Reliability = RELIABLE</b> together with <b>Writer Reliability = BEST_EFFORT</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/reliability/">Reliability</a> and <a href="../../qos/reliability/">Reliability</a>
- What QoS Guard checks: `[Writer.RELIAB < Reader.RELIAB]`

## Example

A best-effort camera publisher and a reliable subscriber on the same topic simply do not connect.

## How to fix it

Make the writer RELIABLE, or relax the reader to BEST_EFFORT. Offered must be at least as strong as requested.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.14 | RELIABILITY | The value offered is considered compatible with the value requested if and only if the inequality "offered kind >= requested kind" evaluates to 'TRUE.' For the purposes of this inequality, the values of RELIABILITY kind are considered ordered such that `BEST_EFFORT < RELIABLE`. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows


Both engines reject endpoint matching when a reader requests `RELIABLE` reliability but the writer only offers `BEST_EFFORT`.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // EDP::valid_matching: offered reliability must satisfy requested reliability.
    if (wdata->m_qos.m_reliability.kind == BEST_EFFORT_RELIABILITY_QOS &&
            rdata->m_qos.m_reliability.kind == RELIABLE_RELIABILITY_QOS)
    {
        incompatible_qos.set(fastdds::dds::RELIABILITY_QOS_POLICY_ID);
    }
    ```

!!! note "Cyclone DDS implementation evidence"
    ```c
    // q_qosmatch.c: reader reliability must not exceed writer reliability.
    if ((mask & QP_RELIABILITY) &&
            rd_qos->reliability.kind > wr_qos->reliability.kind)
    {
        *reason = DDS_RELIABILITY_QOS_POLICY_ID;
        return false;
    }
    ```

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-22/rule-22-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `writer_reliability`, `reader_reliability` |
| Tested values | `writer_reliability ∈ {BEST_EFFORT, RELIABLE}`, `reader_reliability ∈ {BEST_EFFORT, RELIABLE}` |
| Valid / boundary cases | `writer = RELIABLE`, `reader = RELIABLE`; `writer = RELIABLE`, `reader = BEST_EFFORT`; `writer = BEST_EFFORT`, `reader = BEST_EFFORT` |
| Violating cases | `writer = BEST_EFFORT`, `reader = RELIABLE` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `writer_reliability`, `reader_reliability ∈ {BEST_EFFORT, RELIABLE}` | Matched when offered reliability satisfied requested reliability; match rejected when `writer = BEST_EFFORT`, `reader = RELIABLE` |
| Fast DDS 2.14.6 (Jazzy) | `writer_reliability`, `reader_reliability ∈ {BEST_EFFORT, RELIABLE}` | Matched when offered reliability satisfied requested reliability; match rejected when `writer = BEST_EFFORT`, `reader = RELIABLE` |
| Cyclone DDS 0.10.5 | `writer_reliability`, `reader_reliability ∈ {BEST_EFFORT, RELIABLE}` | Matched when offered reliability satisfied requested reliability; match rejected when `writer = BEST_EFFORT`, `reader = RELIABLE` |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as a structural conflict:

```text
[STRUCTURAL] [cmd_vel_pub] Writer.RELIAB <-> Reader.RELIAB (Writer<Reader)
```

At runtime the two endpoints never connect. DDS matches a publisher and subscriber only when the offered QoS satisfies the requested QoS (the Requested-Offered, or RxO, rule). Here the subscriber requests RELIABLE while the publisher offers only BEST_EFFORT, so the offered level is weaker than requested and discovery refuses the match. No connection forms and no samples flow, with no error printed.
