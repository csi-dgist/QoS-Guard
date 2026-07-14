# DURABILITY

Determines how a late-joining Subscriber can receive previously published samples from a Publisher.

## What it controls

How long and where previously published samples remain available for late-joining Readers.

* **VOLATILE**: Does not send any previous samples to newly joined Subscribers.
* **TRANSIENT_LOCAL**: Retains samples in the Publisher's HistoryCache while it is active, allowing late-joining Subscribers to access previously published data.
* **TRANSIENT**: Preserve data after a Publisher has been terminated. Retains data in volatile memory.
* **PERSISTENT**: Preserve data after a Publisher has been terminated. Uses non-volatile storage such as files or databases

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `kind` | volatile | Can **not** be changed at runtime |

## Compatibility role

Matching and late join — Writer offered durability must be at least as strong as Reader requested durability (RxO). Affects replay to late joiners and interacts with lifespan, history, and reader purge settings.

## When this conflicts

Durability takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-03/">
<span class="conflict-title">Best-effort delivery with transient-local durability</span>
<span class="conflict-meta"><span class="conflict-rel">with Reliability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-06/">
<span class="conflict-title">Finite lifespan expires the history that durability should replay</span>
<span class="conflict-meta"><span class="conflict-rel">with Lifespan</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-13/">
<span class="conflict-title">Disposed-sample purge that discards data durability should keep</span>
<span class="conflict-meta"><span class="conflict-rel">with Reader Data Lifecycle</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-19/">
<span class="conflict-title">Manual enable with volatile durability drops pre-enable data</span>
<span class="conflict-meta"><span class="conflict-rel">with Entity Factory</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-20/">
<span class="conflict-title">Partitions filter which retained samples a late joiner replays</span>
<span class="conflict-meta"><span class="conflict-rel">with Partition</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-23/">
<span class="conflict-title">Subscriber requires more durability than the publisher offers</span>
<span class="conflict-meta"><span class="conflict-rel">with itself</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-37/">
<span class="conflict-title">Unbounded durable writer cache under keep-all</span>
<span class="conflict-meta"><span class="conflict-rel">with History</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-40/">
<span class="conflict-title">Durable replay resets the deadline timer for a late joiner</span>
<span class="conflict-meta"><span class="conflict-rel">with Deadline</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
</div>

## Related policies

- [RELIABILITY](reliability.md)
- [LIFESPAN](lifespan.md)
- [ENTITY_FACTORY](entity-factory.md)
- [PARTITION](partition.md)
- [HISTORY](history.md)
- [READER_DATA_LIFECYCLE](reader-data-lifecycle.md)
- [DEADLINE](deadline.md)

### Example

The DURABL QoS can be used to allow newly added or recovered robots to immediately access the same information, thereby improving both system robustness and collaborative efficiency. Data such as global maps or mission plans, whose availability must not depend on the lifespan of a specific robot, should remain accessible to robots that join after the Publisher has terminated or rebooted. In such cases, DURABL should be set to transient if data must persist throughout process restarts, or to persistent if it must survive system-wide reboots.
