# Liveliness lease too tight for the link causes ownership flapping

<p class="rule-ref-line">Rule 39 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes resources. Ordinary delay expires the lease and releases ownership to the backup even though the owner is alive, so ownership oscillates.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>Ownership = EXCLUSIVE</b> together with <b>Liveliness lease shorter than one publish period plus a round-trip</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/liveliness/">Liveliness</a> and <a href="../../qos/ownership/">Ownership</a>
- What QoS Guard checks: `[OWNST = EXCLUSIVE] ∧ [LIVENS.lease < PP + 2×RTT]`

## Example

Lease 40 ms with a 20 ms period and 50 ms RTT. The healthy owner is repeatedly declared lost and ownership flaps.

## How to fix it

Set the liveliness lease_duration larger than PP + 2 x RTT.

## Why this rule is flagged

#### What the DDS specification says

This page settles the rule through the measured failover below rather than a standalone specification clause.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the measurements.

<hr class="evidence-subsection-divider">

#### What the measurements show

On a lossy wireless link (RTT 50 ms, 20% loss), sweeping the liveliness lease shows the share of delivery diverted to a spurious backup writer growing once the lease falls below the loss-inflated recovery window (about one publish period plus two round-trips). The lease is the less sensitive of the two ownership triggers, since it takes a run of consecutive missed assertions to expire it rather than a single delivery gap. Both engines followed the same joint dependence.

| Measurement | Finding |
|---|---|
| Setup | Exclusive ownership on a lossy wireless link (RTT 50 ms, 20% loss), sweeping the liveliness lease |
| Threshold | Delivery diverts to a spurious backup once the lease falls below about one publish period plus two round-trips |
| Sensitivity | The lease is the less sensitive trigger, since it needs a run of consecutive missed assertions |
| Engines | Both engines followed the same joint dependence |
