# No-writer purge that never fires because liveliness never expires

<p class="rule-ref-line">Rule 12 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. With an infinite lease the writer is never marked not-alive, so the no-writer purge timer never starts and samples are kept forever.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Reader autopurge_no_writer_samples_delay is finite</b> together with <b>Liveliness lease_duration = infinite</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/liveliness/">Liveliness</a> and <a href="../../qos/reader-data-lifecycle/">Reader Data Lifecycle</a>
- What QoS Guard checks: `[R.autopurge_nowriter ≠ ∞] ∧ [LIVENS.lease = ∞]`

## Example

You expect samples to be purged shortly after a writer disappears, but the infinite lease means the reader never considers the writer gone.

## How to fix it

Set a finite liveliness lease_duration so writers can be marked not-alive and the no-writer purge can run.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.22 | READER_DATA_LIFECYCLE | `autopurge_nowriter_samples_delay` defines how long the DataReader maintains information for an instance once its `instance_state` becomes `NOT_ALIVE_NO_WRITERS`; after that time elapses, the DataReader purges the internal instance information. |
| §2.2.2.5.1.3 | Instance state | `NOT_ALIVE_NO_WRITERS` means the DataReader has declared the instance not alive because it detected that there are no live DataWriter entities writing that instance. |

Rule 12 is derived from this state transition: if liveliness never declares the writer not alive, the `NOT_ALIVE_NO_WRITERS` state needed to start `autopurge_nowriter_samples_delay` may not be reached.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the specification.

<hr class="evidence-subsection-divider">

#### What the measurements show

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-12/rule-12-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `LIVENS.lease_duration`, `RDLIFE.autopurge_nowriter_ms` |
| Tested values | `LIVENS.lease_duration ∈ {-1 ms, 300 ms}`, `RDLIFE.autopurge_nowriter_ms ∈ {-1 ms, 0 ms, 100 ms}` |
| Rule-relevant case | `LIVENS.lease_duration = -1 ms`, `RDLIFE.autopurge_nowriter_ms ∈ {0 ms, 100 ms}` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `LIVENS.lease_duration ∈ {-1 ms, 300 ms}`, `RDLIFE.autopurge_nowriter_ms ∈ {-1 ms, 0 ms, 100 ms}` | Profile accepted, matched, and delivered |
| Fast DDS 2.14.6 (Jazzy) | `LIVENS.lease_duration ∈ {-1 ms, 300 ms}`, `RDLIFE.autopurge_nowriter_ms ∈ {-1 ms, 0 ms, 100 ms}` | Profile accepted, matched, and delivered; `no_writers = 1` |
| Cyclone DDS 0.10.5 | `LIVENS.lease_duration ∈ {-1 ms, 300 ms}`, `RDLIFE.autopurge_nowriter_ms ∈ {-1 ms, 0 ms, 100 ms}` | Profile accepted, matched, and delivered; `no_writers = 1` |
