# Total sample cap set below the per-instance cap

<p class="rule-ref-line">Rule 2 &middot; applies to publishers and subscribers &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. The two limits contradict each other, and Cyclone DDS rejects the endpoint at creation.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>Resource Limits max_samples = N</b> together with <b>max_samples_per_instance greater than N</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/resource-limits/">Resource Limits</a> and <a href="../../qos/resource-limits/">Resource Limits</a>
- What QoS Guard checks: `[max_samples < max_samples_per_instance]`

## Example

max_samples 5 with max_samples_per_instance 10 is impossible, because a single instance alone could exceed the total budget.

## How to fix it

Set max_samples greater than or equal to max_samples_per_instance.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.19 | RESOURCE_LIMITS | The setting of RESOURCE_LIMITS max_samples must be consistent with the max_samples_per_instance. For these two values to be consistent they must verify that `max_samples >= max_samples_per_instance`. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Fast DDS checks RESOURCE_LIMITS internal consistency in `check_allocation_consistency`. When `max_samples < max_instances * max_samples_per_instance`, creation returns `RETCODE_INCONSISTENT_POLICY`.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // rule 2: check_allocation_consistency
    if ((qos.resource_limits().max_samples > 0) &&
            (qos.resource_limits().max_samples <
            (qos.resource_limits().max_instances * qos.resource_limits().max_samples_per_instance)))
    {
        EPROSIMA_LOG_ERROR(DDS_QOS_CHECK,
            "max_samples should be greater than max_instances * max_samples_per_instance");
        return ReturnCode_t::RETCODE_INCONSISTENT_POLICY;
    }
    ```
    Source: `DataReaderImpl.cpp` (lines 1523–1552).

Cyclone DDS rejected the violating RESOURCE_LIMITS profile at creation time in the tested cases. Fast DDS accepted the same violating profiles in the tested versions.

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-02/rule-02-data.csv) |
| Fixed QoS setting | `RESLIM.mpi = 4` |
| Tested variable | `RESLIM.max_samples` |
| Tested values | `{2, 3, 4, 5}` |
| Valid / boundary cases | `{4, 5}` |
| Violating cases | `{2, 3}` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `max_samples < RESLIM.mpi` | Profile accepted at creation time |
| Fast DDS 2.14.6 (Jazzy) | `max_samples < RESLIM.mpi` | Profile accepted at creation time |
| Cyclone DDS 0.10.5 | `max_samples < RESLIM.mpi` | Profile rejected at creation time |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard flags this as a structural conflict:

```text
[STRUCTURAL] [state_sub] [SUB] max_samples(5) <-> max_samples_per_instance(10)
```

At runtime the two limits contradict each other, because a single instance is allowed 10 samples while the whole reader is capped at 5. Cyclone DDS rejects the endpoint at creation, and Fast DDS returns an inconsistent-policy error from its own consistency check. Either way the reader never starts, so no data is delivered.
