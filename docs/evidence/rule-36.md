# Liveliness lease shorter than the deadline period

<p class="rule-ref-line">Rule 36 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. The writer can be declared not-alive between deadlines, so you get liveliness-lost events that contradict a healthy deadline cadence.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Deadline period is finite</b> together with <b>Liveliness lease_duration shorter than the deadline</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/liveliness/">Liveliness</a> and <a href="../../qos/deadline/">Deadline</a>
- What QoS Guard checks: `[DEADLN.period ≠ ∞] ∧ [LIVENS.lease < DEADLN.period]`

## Example

Lease 50 ms with a 100 ms deadline. The writer is flagged as lost mid-cycle even though it meets every deadline.

## How to fix it

Set the liveliness lease_duration at least as long as the Deadline period.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on direct measurement.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the measurements.

<hr class="evidence-subsection-divider">

#### What the measurements show

Direct measurement confirms that when the liveliness lease is shorter than the deadline period, the subscriber declares the writer not alive between deadline cycles, so a liveliness-lost event fires before the deadline check even runs. This premature liveliness loss preempts deadline monitoring even though the writer is still meeting every deadline. Both engines showed the same crossover once the lease fell below the deadline period.

| Measurement | Finding |
|---|---|
| Setup | Liveliness lease set shorter than the deadline period |
| Result | A liveliness-lost event fires between deadline cycles and preempts deadline monitoring |
| Engines | Both engines showed the same crossover once the lease fell below the deadline period |
