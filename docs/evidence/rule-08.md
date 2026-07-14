# Source-timestamp ordering with a history depth of one

<p class="rule-ref-line">Rule 8 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. With only one slot the reader cannot reorder by source time, so out-of-order samples are dropped. Fast DDS does not support source-timestamp ordering at all.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Destination Order = BY_SOURCE_TIMESTAMP</b> together with <b>History = KEEP_LAST with depth 1</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/history/">History</a> and <a href="../../qos/destination-order/">Destination Order</a>
- What QoS Guard checks: `[DESTORD = BY_SOURCE] ∧ [KEEP_LAST] ∧ [HIST.depth = 1]`

## Example

Two writers publish to one keyed topic with depth 1 and source ordering. A late-arriving but earlier-stamped sample is discarded instead of reordered.

## How to fix it

Increase KEEP_LAST depth, or use KEEP_ALL, when you need source-timestamp ordering. On Fast DDS, avoid BY_SOURCE_TIMESTAMP.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on the engine's implementation.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Fast DDS does not support `BY_SOURCE_TIMESTAMP` for subscribers in the tested versions; profiles were not created or matched.

Cyclone DDS implements source-timestamp ordering in the reader cache. With `KEEP_LAST` depth 1, only one sample is retained, so source ordering has little comparative history to operate on.

!!! note "Cyclone DDS implementation evidence"
    ```c
    // In BY_SOURCE_TIMESTAMP mode, samples that are "reversed relative to the source timestamp (i.e., past-time samples arriving late)" are discarded unconditionally.
    // This behaviour results in retaining only the single most recent sample, rather than performing any sorting function.
    static int inst_accepts_sample (const struct dds_rhc_default *rhc, const struct rhc_instance *inst, const struct ddsi_writer_info *wrinfo, const struct ddsi_serdata sample, const bool has_data)
    {
        if (rhc->by_source_ordering) {
            if (sample->timestamp.v > inst->tstamp.v)
            { /* ok */ }
            else if (sample->timestamp.v < inst->tstamp.v)
            { return 0; }
            else if (inst_accepts_sample_by_writer_guid (inst, wrinfo))
            { /* ok */ }
            else
            { return 0; }
        }
        if (rhc->exclusive_ownership && inst->wr_iid_islive && inst->wr_iid != wrinfo->iid)
        {
            int32_t strength = wrinfo->ownership_strength;
            if (strength > inst->strength) {
                /* ok */
            } else if (strength < inst->strength) {
                return 0;
            } else if (inst_accepts_sample_by_writer_guid (inst, wrinfo)) {
                /* ok */
            } else { return 0; }
        }
        if (has_data && !content_filter_accepts (rhc->reader, sample, inst, wrinfo->iid, inst->iid))
        { return 0; }
        return 1;
    }
    ```

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-08/rule-08-data.csv) |
| Fixed QoS setting | `HIST.kind = KEEP_LAST`, `HIST.depth = 1` |
| Tested variable | `DESTORD.kind` |
| Tested values | `DESTORD ∈ {BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP}` |
| Rule-relevant case | `DESTORD = BY_SOURCE_TIMESTAMP`, `HIST.depth = 1` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT ∈ {1 ms, 50 ms}`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `HIST.kind = KEEP_LAST`, `HIST.depth = 1`, `DESTORD = BY_SOURCE_TIMESTAMP` | Profile not created or matched (`BY_SOURCE_TIMESTAMP` unsupported in the tested Fast DDS versions) |
| Fast DDS 2.14.6 (Jazzy) | `HIST.kind = KEEP_LAST`, `HIST.depth = 1`, `DESTORD = BY_SOURCE_TIMESTAMP` | Profile not created or matched (`BY_SOURCE_TIMESTAMP` unsupported in the tested Fast DDS versions) |
| Cyclone DDS 0.10.5 | `HIST.kind = KEEP_LAST`, `HIST.depth = 1`, `DESTORD = BY_SOURCE_TIMESTAMP` | Profile accepted, matched, and delivered |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard flags this as a functional conflict:

```text
[FUNCTIONAL] [scan_sub] [SUB] DESTORD=BY_SOURCE + HIST.depth≤1 <-> depth≥2
```

At runtime a single history slot leaves nothing to reorder against. In an engine that implements source-timestamp ordering, a late sample carrying an earlier timestamp is discarded rather than reordered, so the ordering guarantee is silently broken. Fast DDS does not support BY_SOURCE_TIMESTAMP at all, so such a profile is rejected at creation instead.
