# Source-timestamp ordering with a per-instance cap of one

<p class="rule-ref-line">Rule 9 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. A single per-instance slot cannot hold enough samples to reorder by source time, so earlier-stamped samples are lost. Fast DDS does not support source-timestamp ordering.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Destination Order = BY_SOURCE_TIMESTAMP with KEEP_ALL</b> together with <b>Resource Limits max_samples_per_instance = 1</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/resource-limits/">Resource Limits</a> and <a href="../../qos/destination-order/">Destination Order</a>
- What QoS Guard checks: `[DESTORD = BY_SOURCE] ∧ [HIST.kind = KEEP_ALL] ∧ [RESLIM.mpi = 1]`

## Example

KEEP_ALL with max_samples_per_instance 1 and source ordering behaves like depth 1, which defeats reordering.

## How to fix it

Raise max_samples_per_instance above 1 when using source-timestamp ordering. On Fast DDS, avoid BY_SOURCE_TIMESTAMP.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on the engine's implementation.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Fast DDS does not support `BY_SOURCE_TIMESTAMP` for subscribers in the tested versions; profiles were not created or matched.

Cyclone DDS implements source-timestamp ordering in the reader cache. With `KEEP_ALL` and `max_samples_per_instance = 1`, only one sample is retained per instance, so source ordering has little comparative history to operate on.

!!! note "Cyclone DDS implementation evidence"
    ```c
    // Set to Keep_all, but limited to 1 due to max_samples_per_instance
    /* Check if resource max_samples QoS exceeded */
    if (rhc->reader && rhc->max_samples != DDS_LENGTH_UNLIMITED && rhc->n_vsamples >= (uint32_t) rhc->max_samples)
    {
        cb_data->raw_status_id = (int) DDS_SAMPLE_REJECTED_STATUS_ID;
        cb_data->extra = DDS_REJECTED_BY_SAMPLES_LIMIT;
        cb_data->handle = inst->iid;
        cb_data->add = true; return false;
    }
    /* Check if resource max_samples_per_instance QoS exceeded */
    if (rhc->reader && rhc->max_samples_per_instance != DDS_LENGTH_UNLIMITED && inst->nvsamples >= (uint32_t) rhc->max_samples_per_instance)
    {
        cb_data->raw_status_id = (int) DDS_SAMPLE_REJECTED_STATUS_ID;
        cb_data->extra = DDS_REJECTED_BY_SAMPLES_PER_INSTANCE_LIMIT;
        cb_data->handle = inst->iid;
        cb_data->add = true; return false;
    }
    ```

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-09/rule-09-data.csv) |
| Fixed QoS setting | `HIST.kind = KEEP_ALL` |
| Tested variable | `RESLIM.mpi`, `DESTORD.kind` |
| Tested values | `RESLIM.mpi ∈ {0, 1, 2}`, `DESTORD ∈ {BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP}` |
| Rule-relevant case | `DESTORD = BY_SOURCE_TIMESTAMP`, `HIST.kind = KEEP_ALL` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT ∈ {1 ms, 50 ms}`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `HIST.kind = KEEP_ALL`, `RESLIM.mpi ∈ {0, 1, 2}`, `DESTORD = BY_SOURCE_TIMESTAMP` | Profile not created or matched (`BY_SOURCE_TIMESTAMP` unsupported in the tested Fast DDS versions) |
| Fast DDS 2.14.6 (Jazzy) | `HIST.kind = KEEP_ALL`, `RESLIM.mpi ∈ {0, 1, 2}`, `DESTORD = BY_SOURCE_TIMESTAMP` | Profile not created or matched (`BY_SOURCE_TIMESTAMP` unsupported in the tested Fast DDS versions) |
| Cyclone DDS 0.10.5 | `HIST.kind = KEEP_ALL`, `RESLIM.mpi ∈ {0, 1, 2}`, `DESTORD = BY_SOURCE_TIMESTAMP` | Profile accepted, matched, and delivered for `RESLIM.mpi ∈ {1, 2}`; no delivery for `RESLIM.mpi = 0` |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard flags this as a functional conflict:

```text
[FUNCTIONAL] [cloud_sub] [SUB] DESTORD=BY_SOURCE + KEEP_ALL + mpi=1 <-> reorder buffer
```

At runtime a per-instance cap of one makes KEEP_ALL behave like a depth of one, so the reader holds a single sample per instance. An engine that orders by source timestamp then has no earlier sample to compare against, and a late but earlier-stamped update is dropped instead of reordered. Fast DDS does not support BY_SOURCE_TIMESTAMP, so the profile is rejected at creation instead.
