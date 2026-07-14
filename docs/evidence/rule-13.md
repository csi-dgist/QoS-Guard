# Disposed-sample purge that discards data durability should keep

<p class="rule-ref-line">Rule 13 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. The reader purges disposed samples on a timer, which can drop history that durability is meant to preserve.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Durability = TRANSIENT or stronger</b> together with <b>Reader autopurge_disposed_samples_delay is finite</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/reader-data-lifecycle/">Reader Data Lifecycle</a> and <a href="../../qos/durability/">Durability</a>
- What QoS Guard checks: `[DURABL ≥ TRANSIENT] ∧ [R.autopurge_disposed ≠ ∞]`

## Example

A durable topic is expected to retain history, but a short disposed-sample purge delay removes it early.

## How to fix it

Use an infinite autopurge_disposed_samples_delay when you rely on TRANSIENT durability to retain history.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.22 | READER_DATA_LIFECYCLE | `autopurge_disposed_samples_delay` defines how long the DataReader maintains information for an instance once its `instance_state` becomes `NOT_ALIVE_DISPOSED`; after that time elapses, the DataReader may purge the internal instance information. |
| §2.2.3.4 | DURABILITY | `TRANSIENT_LOCAL` or stronger durability allows data to be retained for late-joining readers according to the writer-side durability behavior. |
| §2.2.2.5.1.3 | Instance state | `NOT_ALIVE_DISPOSED` indicates that the instance has been explicitly disposed and is no longer alive. |

Rule 13 is derived from this interaction: finite reader-side disposed-sample autopurge can remove disposed-instance information while durable history expects retained state to remain available for replay.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the specification.

<hr class="evidence-subsection-divider">

#### What the measurements show

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-13/rule-13-data.csv) |
| Fixed QoS setting | `DURABL = TRANSIENT_LOCAL` |
| Tested variable | `RDLIFE.autopurge_disposed_ms` |
| Tested values | `RDLIFE.autopurge_disposed_ms ∈ {-1 ms, 0 ms, 100 ms}` |
| Rule-relevant case | `DURABL = TRANSIENT_LOCAL`, `RDLIFE.autopurge_disposed_ms ∈ {0 ms, 100 ms}` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `DURABL = TRANSIENT_LOCAL`, `RDLIFE.autopurge_disposed_ms ∈ {-1 ms, 0 ms, 100 ms}` | Profile accepted, matched, and delivered |
| Fast DDS 2.14.6 (Jazzy) | `DURABL = TRANSIENT_LOCAL`, `RDLIFE.autopurge_disposed_ms ∈ {-1 ms, 0 ms, 100 ms}` | Profile accepted, matched, and delivered; `disposed = 1`, `no_writers = 1` |
| Cyclone DDS 0.10.5 | `DURABL = TRANSIENT_LOCAL`, `RDLIFE.autopurge_disposed_ms ∈ {-1 ms, 0 ms, 100 ms}` | Profile accepted, matched, and delivered; `disposed = 1`, `no_writers = 1` |
