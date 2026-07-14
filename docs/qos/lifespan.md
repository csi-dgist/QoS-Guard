# LIFESPAN

Controls how long a sample published by a Publisher remains valid.

## What it controls

The expiration duration for samples in the Writer HistoryCache. Expired samples are not delivered to new or existing Readers.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `duration` | infinity | Can be changed at runtime |

## Compatibility role

Timing and cache — limits how long samples remain deliverable. Interacts with deadline period, history depth, resource limits, durability, and reliable delivery windows.

## When this conflicts

Lifespan takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-06/">
<span class="conflict-title">Finite lifespan expires the history that durability should replay</span>
<span class="conflict-meta"><span class="conflict-rel">with Durability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-07/">
<span class="conflict-title">Lifespan shorter than the deadline period</span>
<span class="conflict-meta"><span class="conflict-rel">with Deadline</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-17/">
<span class="conflict-title">Lifespan longer than the history window can ever hold</span>
<span class="conflict-meta"><span class="conflict-rel">with History</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-18/">
<span class="conflict-title">Lifespan longer than the resource-limit buffer can ever hold</span>
<span class="conflict-meta"><span class="conflict-rel">with Resource Limits</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-33/">
<span class="conflict-title">Lifespan expires samples before reliable retransmission finishes</span>
<span class="conflict-meta"><span class="conflict-rel">with Reliability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
</div>

## Related policies

- [DURABILITY](durability.md)
- [DEADLINE](deadline.md)
- [HISTORY](history.md)
- [RESOURCE_LIMITS](resource-limits.md)
- [RELIABILITY](reliability.md)

### Example

The LFSPAN QoS can be used to prevent robots from retaining outdated samples unnecessarily. For data such as position or battery level, where only the last few seconds matter, setting the duration accordingly ensures that the Publisher's HistoryCache stores only the most recent samples, with older ones automatically deleted to conserve memory. In contrast, for data such as command logs, where long-term delivery is more important than freshness, it is preferable to set duration to infinity and rely on other QoS policies to ensure reliability.
