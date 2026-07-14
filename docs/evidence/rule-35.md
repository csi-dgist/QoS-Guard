# Best-effort loss triggers false deadline-missed events

<p class="rule-ref-line">Rule 35 &middot; applies to the subscriber &middot; <a href="../../rules/">Back to all rules</a></p>

Breaks a guarantee. Dropped samples create gaps that look like missed deadlines, even when the writer published on time.

<div class="rule-conflict-callout rule-conflict-guarantee">
<div class="rule-conflict-settings">If you set <b>Reliability = BEST_EFFORT</b> together with <b>Deadline period is finite</b></div>
<div class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</div>
</div>

- Settings involved: <a href="../../qos/reliability/">Reliability</a> and <a href="../../qos/deadline/">Deadline</a>
- What QoS Guard checks: `[DEADLN.period ≠ ∞] ∧ [RELIAB = BEST_EFFORT]`

## Example

A 100 ms deadline over a lossy best-effort link fires requested-deadline-missed whenever a sample is dropped.

## How to fix it

Use RELIABLE on deadline-monitored topics, or widen the deadline to tolerate best-effort loss.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on direct measurement.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

The behavior here does not depend on a specific engine's implementation, so the rule follows from the measurements.

<hr class="evidence-subsection-divider">

#### What the measurements show

Direct measurement confirms that under best-effort reliability a lost sample is never retransmitted, so the gap it leaves is read by the subscriber as a missed deadline even though the writer published on schedule. The deadline monitor cannot distinguish a dropped sample from a writer that has fallen silent, so requested-deadline-missed events fire whenever loss occurs. Both engines behaved the same way in the tested runs.

| Measurement | Finding |
|---|---|
| Setup | Best-effort reliability with a finite deadline, under packet loss |
| Result | Each dropped sample triggers a requested-deadline-missed event even though the writer published on time |
| Engines | Fast DDS and Cyclone DDS behaved the same |
