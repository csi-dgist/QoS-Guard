# Immediate no-writer purge with auto-dispose turned off

<p class="rule-ref-line">Rule 28 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. Samples are purged the instant a writer goes not-alive, before the intended dispose-based cleanup can run.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Writer autodispose_unregistered_instances = false</b> together with <b>Reader autopurge_no_writer_samples_delay = 0</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/writer-data-lifecycle/">Writer Data Lifecycle</a> and <a href="../../qos/reader-data-lifecycle/">Reader Data Lifecycle</a>
- What QoS Guard checks: `[W.autodispose = FALSE] ∧ [R.autopurge_nowriter = 0]`

## Example

With auto-dispose off and a zero no-writer delay, a brief writer dropout wipes the reader's cache immediately.

## How to fix it

Use a non-zero no-writer purge delay, or enable auto-dispose so cleanup is driven by dispose rather than by writer liveliness.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.21 | WRITER_DATA_LIFECYCLE | If `autodispose_unregistered_instances` is false, unregistering a writer does not dispose the instance. |
| §2.2.3.22 | READER_DATA_LIFECYCLE | `autopurge_nowriter_samples_delay` defines how long a DataReader maintains information for an instance once its state becomes `NOT_ALIVE_NO_WRITERS`. |
| §2.2.2.5.1.3 | Instance state | `NOT_ALIVE_NO_WRITERS` indicates that there are no live DataWriter entities writing the instance. |

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-28/rule-28-data.csv) |
| Fixed QoS setting | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = 0` |
| Tested variable | `probe` |
| Tested values | `probe = no_writers_purge` |
| Rule-relevant case | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = 0` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = 0`, `probe = no_writers_purge` | Profile accepted, matched, and delivered |
| Fast DDS 2.14.6 (Jazzy) | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = 0`, `probe = no_writers_purge` | Profile accepted, matched, and delivered; `no_writers = 1`, `disposed = 0` |
| Cyclone DDS 0.10.5 | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_nowriter_ms = 0`, `probe = no_writers_purge` | Profile accepted, matched, and delivered; `no_writers = 1`, `disposed = 0` |

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the specification.

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as a functional conflict:

```text
[FUNCTIONAL] W.autodispose=FALSE <-> R.autopurge_nowriter=0. Samples may never be purged when all writers disappear.
```

At runtime the reader treats a writer going not-alive as the trigger to reclaim its instances, and a zero no-writer delay reclaims them immediately. Because auto-dispose is off, those instances were never disposed, so the dispose-based cleanup you intended never runs. A brief writer dropout then wipes the reader's cache at once, breaking the retention you expected.
