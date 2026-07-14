# Durable replay resets the deadline timer for a late joiner

<p class="rule-ref-line">Rule 40 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes effort and can mislead monitoring. Replayed historical samples reset the reader's deadline timer, so the deadline reflects replay timing rather than the live publishing cadence.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>Durability = TRANSIENT_LOCAL or stronger</b> together with <b>Deadline period is finite</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/durability/">Durability</a> and <a href="../../qos/deadline/">Deadline</a>
- What QoS Guard checks: `[DEADLN.period ≠ ∞] ∧ [DURABL ≥ TRAN_LOCAL]`

## Example

A late joiner receives a burst of retained samples, which resets its deadline clock and briefly hides a genuinely slow publisher.

## How to fix it

Expect deadline timing to be dominated by replay right after a late join, and judge live cadence only once replay settles.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on the engine's implementation and direct measurement.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Cyclone DDS renews the reader deadline timer from the normal sample insertion path. Historical samples replayed by `TRANSIENT_LOCAL` durability enter that same path, so replay can refresh deadline tracking.

!!! note "Cyclone DDS implementation evidence"
    ```c
    // dds_rhc_default.c: postprocess_instance_update renews deadline on sample insertion.
    static void postprocess_instance_update(
            struct dds_rhc_default *rhc,
            struct rhc_instance **instptr,
            const struct trigger_info_pre *pre,
            const struct trigger_info_post *post,
            struct trigger_info_qcond *trig_qc)
    {
        struct rhc_instance *inst = *instptr;
    #ifdef DDS_HAS_DEADLINE_MISSED
        if (inst->deadline_reg)
        {
            deadline_renew_instance_locked(&rhc->deadline, &inst->deadline);
        }
        else
        {
            deadline_register_instance_locked(
                    &rhc->deadline, &inst->deadline, ddsrt_time_monotonic());
            inst->deadline_reg = 1;
        }
    #endif
    }
    ```

<hr class="evidence-subsection-divider">

#### What the measurements show

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-40/rule-40-data.csv) |
| Fixed QoS setting | `DURABL = TRANSIENT_LOCAL` |
| Tested variable | `DEADLN.period` |
| Tested values | `DEADLN.period ∈ {-1 ms, 100 ms, 200 ms}` |
| Rule-relevant case | `DURABL = TRANSIENT_LOCAL` with late-joining reader replay under tested deadline settings |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT ∈ {1 ms, 50 ms}`, `loss ∈ {0%, 20%}`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `DURABL = TRANSIENT_LOCAL`, `DEADLN.period ∈ {-1 ms, 100 ms, 200 ms}` | Profile accepted and delivered in the Humble measurement summary |
| Fast DDS 2.14.6 (Jazzy) | `DURABL = TRANSIENT_LOCAL`, `DEADLN.period ∈ {-1 ms, 100 ms, 200 ms}` | Late-join replay was recovered in most tested cases; some no-delivery runs occurred for `DEADLN.period = -1 ms` and `DEADLN.period = 200 ms` |
| Cyclone DDS 0.10.5 | `DURABL = TRANSIENT_LOCAL`, `DEADLN.period ∈ {-1 ms, 100 ms, 200 ms}` | Late-join replay was recovered in all tested cases |
