# History depth set deeper than the resource-limit cap

<p class="rule-ref-line">Rule 1 &middot; applies to publishers and subscribers &middot; <a href="../../rules/">Back to all rules</a></p>

Won't connect. Cyclone DDS rejects the endpoint at creation. Fast DDS accepts it but can only keep up to the cap, so the extra depth is a lie.

<div class="rule-conflict-callout rule-conflict-connect">
<div class="rule-conflict-settings">If you set <b>History = KEEP_LAST with depth D</b> together with <b>Resource Limits max_samples_per_instance smaller than D</b></div>
<div class="rule-consequence rule-consequence-connect">Won't connect</div>
</div>

- Settings involved: <a href="../../qos/history/">History</a> and <a href="../../qos/resource-limits/">Resource Limits</a>
- What QoS Guard checks: `[HIST.kind = KEEP_LAST] ∧ [HIST.depth > RESLIM.mpi]`

## Example

You set KEEP_LAST depth 10 but max_samples_per_instance 4. Cyclone refuses to create the reader, and Fast DDS silently keeps at most 4 samples.

## How to fix it

Keep depth less than or equal to max_samples_per_instance, or raise max_samples_per_instance to match the depth you actually need.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.18 | HISTORY | The setting of HISTORY depth must be consistent with the RESOURCE_LIMITS max_samples_per_instance. For these two QoS to be consistent, they must verify that `depth <= max_samples_per_instance`. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Fast DDS compares `HISTORY.depth` against `RESOURCE_LIMITS.max_samples_per_instance` during reader QoS validation. When `depth > max_samples_per_instance`, an inconsistency warning is emitted.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // rule 1: HISTORY.depth vs RESOURCE_LIMITS.max_samples_per_instance
    if (qos.history().kind == KEEP_LAST_HISTORY_QOS && qos.history().depth > 0 &&
            qos.resource_limits().max_samples_per_instance > 0 &&
            qos.history().depth > qos.resource_limits().max_samples_per_instance)
    {
        EPROSIMA_LOG_WARNING(RTPS_QOS_CHECK,
            "HISTORY DEPTH '" << qos.history().depth <<
            "' is inconsistent with max_samples_per_instance ... depth <= max_samples_per_instance.");
    }
    ```
    Source: `DataReaderImpl.cpp` (lines 1523–1552).

Cyclone DDS rejected the profile at creation time when `KEEP_LAST` depth exceeded `max_samples_per_instance`, while Fast DDS accepted the same violating profiles in the tested versions.

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-01/rule-01-data.csv) |
| Fixed QoS setting | `HIST.kind = KEEP_LAST`, `RESLIM.mpi = 4` |
| Tested variable | `HIST.depth` |
| Tested values | `{2, 3, 4, 5, 8}` |
| Valid / boundary cases | `{2, 3, 4}` |
| Violating cases | `{5, 8}` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `KEEP_LAST`, `depth > mpi` | Profile accepted at creation time |
| Fast DDS 2.14.6 (Jazzy) | `KEEP_LAST`, `depth > mpi` | Profile accepted at creation time |
| Cyclone DDS 0.10.5 | `KEEP_LAST`, `depth > mpi` | Profile rejected at creation time |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard flags this as a structural conflict:

```text
[STRUCTURAL] [scan_sub] [SUB] KEEP_LAST(depth=20) <-> max_samples_per_instance(10)
```

At runtime these two settings cannot both hold. Cyclone DDS rejects the endpoint at creation, so it never starts. Fast DDS accepts the profile but its resource-limit cap keeps only 10 samples per instance, silently truncating the history depth of 20 that the reader asked for.
