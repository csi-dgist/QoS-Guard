# Auto-dispose on unregister races exclusive-ownership failover

<p class="rule-ref-line">Rule 16 &middot; applies to the publisher &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. When the owner unregisters, auto-dispose can dispose the instance before ownership fails over, so the backup writer's data is treated as disposed.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Ownership = EXCLUSIVE</b> together with <b>Writer autodispose_unregistered_instances = true</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/ownership/">Ownership</a> and <a href="../../qos/writer-data-lifecycle/">Writer Data Lifecycle</a>
- What QoS Guard checks: `[W.autodispose = TRUE] ∧ [OWNST = EXCLUSIVE]`

## Example

The primary writer exits and auto-disposes the instance. The reader marks it disposed instead of switching to the still-live backup writer.

## How to fix it

Disable autodispose_unregistered_instances on exclusive-ownership topics that use redundant writers.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on the engine's implementation and direct measurement.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Under exclusive ownership, the current owner state is cleared when the owning writer is unregistered. The autodispose/unregister path therefore interacts with ownership failover: once the current owner is cleared, another writer can become the owner.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // writer_unregister(): owner clear and re-selection
    if (writer_guid == current_owner.first)
    {
        current_owner.second = 0;
        current_owner.first  = fastrtps::rtps::c_Guid_Unknown;

        if (ALIVE_INSTANCE_STATE == instance_state)
        {
            update_owner();
        }
    }
    ```

    Source: `DataReaderInstance.hpp` writer unregister / ownership update path.

!!! note "Cyclone DDS implementation evidence"
    ```c
    // unregister path: unregister the writer from the reader history cache
    if (delta < 0 && rd->rhc)
    {
        struct ddsi_writer_info wrinfo;
        ddsi_make_writer_info(&wrinfo, &pwr->e, pwr->c.xqos, DDSI_STATUSINFO_UNREGISTER);
        ddsi_rhc_unregister_wr(rd->rhc, &wrinfo);
    }
    ```

    ```c
    // unregister releases the live-owner marker
    if (inst->wr_iid_islive && wrinfo->iid == inst->wr_iid)
    {
        inst->wr_iid_islive = 0;
        inst->strength = 0;
    }
    ```

    Source: `ddsi_endpoint.c` unregister path and `dds_rhc_default.c` ownership-state release path.

In both implementations, unregistering the current writer clears the ownership state under exclusive ownership. With `autodispose_unregistered_instances = true`, this writer-lifecycle path can therefore trigger ownership failover behavior.

<hr class="evidence-subsection-divider">

#### What the measurements show

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-16/rule-16-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `OWNST.kind`, `WDLIFE.autodispose_unregistered_instances` |
| Tested values | `OWNST ∈ {SHARED, EXCLUSIVE}`, `WDLIFE.autodispose ∈ {true, false}` |
| Rule-relevant case | `OWNST = EXCLUSIVE`, `WDLIFE.autodispose = true` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `OWNST ∈ {SHARED, EXCLUSIVE}`, `WDLIFE.autodispose ∈ {true, false}` | Profile accepted, matched, and delivered; ownership failover observed |
| Fast DDS 2.14.6 (Jazzy) | `OWNST ∈ {SHARED, EXCLUSIVE}`, `WDLIFE.autodispose ∈ {true, false}` | Profile accepted, matched, and delivered; ownership failover observed with average latency around 385 ms |
| Cyclone DDS 0.10.5 | `OWNST ∈ {SHARED, EXCLUSIVE}`, `WDLIFE.autodispose ∈ {true, false}` | Profile accepted, matched, and delivered; ownership failover observed with average latency around 37 ms |
