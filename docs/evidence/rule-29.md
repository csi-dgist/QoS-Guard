# Disposed-sample purge that can never fire

<p class="rule-ref-line">Rule 29 &middot; applies to publisher and subscriber matching &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes resources. Instances may never enter the disposed state, so the disposed-purge timer never starts and the setting has no effect.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>Writer autodispose_unregistered_instances = false</b> together with <b>Reader autopurge_disposed_samples_delay is finite</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/writer-data-lifecycle/">Writer Data Lifecycle</a> and <a href="../../qos/reader-data-lifecycle/">Reader Data Lifecycle</a>
- What QoS Guard checks: `[W.autodispose = FALSE] ∧ [R.autopurge_disposed ≠ ∞]`

## Example

You set a disposed-sample purge delay expecting cleanup, but with auto-dispose off nothing ever becomes disposed.

## How to fix it

Enable auto-dispose if you want disposed-sample purging to run, or drive cleanup with the no-writer delay instead.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.21 | WRITER_DATA_LIFECYCLE | If `autodispose_unregistered_instances` is false, unregistering a writer does not dispose the instance. |
| §2.2.3.22 | READER_DATA_LIFECYCLE | `autopurge_disposed_samples_delay` defines how long a DataReader maintains information for an instance once its state becomes `NOT_ALIVE_DISPOSED`. |
| §2.2.2.5.1.3 | Instance state | `NOT_ALIVE_DISPOSED` requires the instance to be disposed, not merely left with no live writers. |

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-29/rule-29-data.csv) |
| Fixed QoS setting | `WDLIFE.autodispose = false` |
| Tested variable | `RDLIFE.autopurge_disposed_ms` |
| Tested values | `RDLIFE.autopurge_disposed_ms ∈ {50 ms, 100 ms, 200 ms}` |
| Rule-relevant case | `WDLIFE.autodispose = false`, finite `RDLIFE.autopurge_disposed_ms` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_disposed_ms ∈ {50 ms, 100 ms, 200 ms}` | Profile accepted, matched, and delivered |
| Fast DDS 2.14.6 (Jazzy) | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_disposed_ms ∈ {50 ms, 100 ms, 200 ms}` | Profile accepted, matched, and delivered; `disposed = 1`, `no_writers = 1` |
| Cyclone DDS 0.10.5 | `WDLIFE.autodispose = false`, `RDLIFE.autopurge_disposed_ms ∈ {50 ms, 100 ms, 200 ms}` | Profile accepted, matched, and delivered; `disposed = 1`, `no_writers = 1` |

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the specification.

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard reports this mismatch as an operational conflict:

```text
[OPERATIONAL] WDLIFE.autodispose=FALSE <-> RDLIFE.autopurge_disposed>0
```

At runtime the disposed-sample purge timer only starts once an instance actually enters the disposed state. With auto-dispose turned off, unregistering a writer never disposes the instance, so nothing ever becomes disposed. The finite disposed-purge delay you configured therefore never fires and the setting has no effect.
