# Deadline too tight for the link forces needless ownership handover

<p class="rule-ref-line">Rule 38 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes resources. Normal network delay misses the tight deadline and hands ownership to the backup writer even though the primary is healthy, causing flapping.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>Ownership = EXCLUSIVE</b> together with <b>Deadline shorter than one publish period plus a round-trip</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/deadline/">Deadline</a> and <a href="../../qos/ownership/">Ownership</a>
- What QoS Guard checks: `[OWNST = EXCLUSIVE] ∧ [DEADLN.period < PP + 2×RTT]`

## Example

Deadline 40 ms with a 20 ms period and 50 ms RTT. Ownership keeps bouncing between primary and backup under ordinary jitter.

## How to fix it

Set the Deadline period larger than PP + 2 x RTT so transient delay does not trigger failover.

## Why this rule is flagged

#### What the DDS specification says

This page settles the rule through the measured failover below rather than a standalone specification clause.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the measurements.

<hr class="evidence-subsection-divider">

#### What the measurements show

On a lossy wireless link (RTT 50 ms, 20% loss), sweeping the deadline period shows the share of delivery diverted to a spurious backup writer growing once the deadline period falls below the loss-inflated recovery window (about one publish period plus two round-trips). The deadline is the more sensitive of the two ownership triggers, since a single delivery gap is enough to trip it. Both engines showed the same dependence, though Fast DDS diverted two to three times as much delivery to the backup as Cyclone DDS in the affected region.

| Measurement | Finding |
|---|---|
| Setup | Exclusive ownership on a lossy wireless link (RTT 50 ms, 20% loss), sweeping the deadline period |
| Threshold | Delivery diverts to a spurious backup once the deadline period falls below about one publish period plus two round-trips |
| Sensitivity | The deadline is the more sensitive trigger, since a single delivery gap trips it |
| Engines | Fast DDS diverts two to three times as much delivery to the backup as Cyclone DDS |
