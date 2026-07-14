# Lifespan longer than the history window can ever hold

<p class="rule-ref-line">Rule 17 &middot; applies to publishers and subscribers &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes effort. History already drops old samples once depth is exceeded, so the extra Lifespan validity never applies.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>History = KEEP_LAST with depth D</b> together with <b>Lifespan longer than D publish periods</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/history/">History</a> and <a href="../../qos/lifespan/">Lifespan</a>
- What QoS Guard checks: `[HIST.kind = KEEP_LAST] ∧ [LFSPAN.duration > HIST.depth × PP]`

## Example

Depth 5 at a 20 ms publish period holds about 100 ms of data, but Lifespan is 1 s. The last 900 ms of validity is meaningless.

## How to fix it

Size Lifespan and history together. Either raise depth or lower Lifespan so they describe the same retention window.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on direct measurement.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

This page grounds the rule in the measurement below rather than a separate source trace.

<hr class="evidence-subsection-divider">

#### What the measurements show

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-17/rule-17-data.csv) |
| Fixed QoS setting | `HIST.kind = KEEP_LAST`, `HIST.depth = 3`, `DURABL = TRANSIENT_LOCAL` |
| Tested variable | `LIFESPAN.duration` |
| Tested values | `LIFESPAN.duration ∈ {50 ms, 100 ms, 150 ms, 200 ms, 250 ms}` |
| Rule boundary | `HIST.depth × PP = 3 × 50 ms = 150 ms` |
| Rule-relevant case | `LIFESPAN.duration > HIST.depth × PP`, i.e., `LIFESPAN.duration ∈ {200 ms, 250 ms}` |
| Tested engines / versions | Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT ∈ {1 ms, 50 ms}`, `loss ∈ {0%, 20%}`, `PP = 50 ms`, `message size = 1024 B` |
| Primary metric | `late_join_recovered` |
| Supplementary metric | `delivery_ratio` |

#### Measurement result

##### Main late-join recovery view

![Rule 17 CycloneDDS late-join recovery heatmap](../figures/rule-17/rule17-cyclonedds-1.png)

![Rule 17 FastDDS late-join recovery heatmap](../figures/rule-17/rule17-fastdds-1.png)

The heatmaps show late-join recovery as `LIFESPAN.duration` crosses the `HIST.depth × PP = 150 ms` window.

##### Supplementary delivery-ratio view

![Rule 17 CycloneDDS delivery-ratio heatmap](../figures/rule-17/rule17_heatmap_cyclonedds.png)

![Rule 17 FastDDS delivery-ratio heatmap](../figures/rule-17/rule17_heatmap_fastdds.png)

The supplementary heatmaps show average delivery ratio under the same settings; the primary Rule 17 evidence is the late-join recovery view above.
