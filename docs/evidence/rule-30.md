# Reader cache that is never reclaimed after writers leave

<p class="rule-ref-line">Rule 30 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes resources. With neither dispose nor a finite no-writer delay, samples from departed writers are retained forever and memory grows without bound.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>Writer autodispose_unregistered_instances = false</b> together with <b>Reader autopurge_no_writer_samples_delay = infinite</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/writer-data-lifecycle/">Writer Data Lifecycle</a> and <a href="../../qos/reader-data-lifecycle/">Reader Data Lifecycle</a>
- What QoS Guard checks: `[W.autodispose = FALSE] ∧ [R.autopurge_nowriter = ∞]`

## Example

Writers come and go over hours. The reader keeps every instance from every writer, because nothing ever purges them.

## How to fix it

Set a finite no-writer purge delay, or enable auto-dispose, so stale instances are reclaimed.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.21 | WRITER_DATA_LIFECYCLE | If `autodispose_unregistered_instances` is false, unregistering a writer does not dispose the instance. |
| §2.2.3.22 | READER_DATA_LIFECYCLE | `autopurge_nowriter_samples_delay` defines how long a DataReader maintains information for an instance once its state becomes `NOT_ALIVE_NO_WRITERS`. |
| §2.2.2.5.1.3 | Instance state | `NOT_ALIVE_NO_WRITERS` indicates that there are no live DataWriter entities writing the instance. |

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-30/rule-30-data.csv) |
| Fixed QoS setting | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = -1` |
| Tested variable | `probe` |
| Tested values | `probe = no_writers_persist` |
| Rule-relevant case | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = -1` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = -1`, `probe = no_writers_persist` | Profile accepted, matched, and delivered |
| Fast DDS 2.14.6 (Jazzy) | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = -1`, `probe = no_writers_persist` | Profile accepted, matched, and delivered; `no_writers = 1`, `disposed = 0` |
| Cyclone DDS 0.10.5 | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = -1`, `probe = no_writers_persist` | Profile accepted, matched, and delivered; `no_writers = 1`, `disposed = 0` |

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the specification.

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as an operational conflict:

```text
[OPERATIONAL] W.autodispose=FALSE <-> R.autopurge_nowriter=∞. Samples may never be purged when all writers disappear.
```

At runtime an infinite no-writer delay tells the reader to keep instances forever after their writers leave, and with auto-dispose off those instances are never disposed either. Neither cleanup path can fire, so samples from writers that departed long ago are retained indefinitely. The reader's cache grows without bound as writers come and go.
