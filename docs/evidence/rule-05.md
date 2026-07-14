# Best-effort delivery with manual liveliness

<p class="rule-ref-line">Rule 5 &middot; applies to publishers and subscribers &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. Liveliness assertions can be lost, so the reader may report the writer as lost even though it is alive.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Reliability = BEST_EFFORT</b> together with <b>Liveliness = MANUAL_BY_TOPIC</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/reliability/">Reliability</a> and <a href="../../qos/liveliness/">Liveliness</a>
- What QoS Guard checks: `[LIVENS = MANUAL] ∧ [RELIAB = BEST_EFFORT]`

## Example

A heartbeat publisher asserts liveliness manually over best-effort. Under packet loss the subscriber sees false liveliness-lost events.

## How to fix it

Use RELIABLE when liveliness is asserted manually, so the assertions are retransmitted instead of dropped.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on direct measurement.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the measurements.

<hr class="evidence-subsection-divider">

#### What the measurements show

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-05/rule-05-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `RELIAB.kind`, `LIVENS.kind`, `LIVENS.lease_duration` |
| Tested values | `RELIAB ∈ {BEST_EFFORT, RELIABLE}`, `LIVENS ∈ {AUTOMATIC, MANUAL_BY_TOPIC}`, `lease_duration ∈ {150 ms, 300 ms}` |
| Rule-relevant case | `RELIAB = BEST_EFFORT`, `LIVENS = MANUAL_BY_TOPIC` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 1024 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `RELIAB = BEST_EFFORT`, `LIVENS = MANUAL_BY_TOPIC` | Profile accepted, matched, and delivered. |
| Fast DDS 2.14.6 (Jazzy) | `RELIAB = BEST_EFFORT`, `LIVENS = MANUAL_BY_TOPIC` | Profile accepted, matched, and delivered; `liveliness_lost = 2`, `reader_not_alive = 2` |
| Cyclone DDS 0.10.5 | `RELIAB = BEST_EFFORT`, `LIVENS = MANUAL_BY_TOPIC` | Profile creation/matching failed; `reader_not_alive = 1` |
