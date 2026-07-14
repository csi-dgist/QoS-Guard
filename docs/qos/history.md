# HISTORY

Determines how many samples a Publisher retains in its HistoryCache for retransmission, and how many samples a Subscriber stores before they are delivered to the application.

## What it controls

How many samples per instance are kept in Writer and Reader caches.

* **KEEP_LAST**: Retains only the most recent samples per instance, with depth specifying the maximum number of samples to keep.
* **KEEP_ALL**: Stores all samples for each instance and attempts to deliver as many as possible.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `kind` | KEEP_LAST, depth=1 | Can **not** be changed at runtime |
| `depth` | 1 (when KEEP_LAST) | Can **not** be changed at runtime |

## Compatibility role

Delivery and cache — governs retransmission buffer depth on the Writer and receive queue on the Reader. Interacts with resource limits, lifespan, durability, and destination order.

## When this conflicts

History takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-01/">
<span class="conflict-title">History depth set deeper than the resource-limit cap</span>
<span class="conflict-meta"><span class="conflict-rel">with Resource Limits</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-08/">
<span class="conflict-title">Source-timestamp ordering with a history depth of one</span>
<span class="conflict-meta"><span class="conflict-rel">with Destination Order</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-17/">
<span class="conflict-title">Lifespan longer than the history window can ever hold</span>
<span class="conflict-meta"><span class="conflict-rel">with Lifespan</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-31/">
<span class="conflict-title">Reliable history too shallow for the round-trip</span>
<span class="conflict-meta"><span class="conflict-rel">with Reliability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-37/">
<span class="conflict-title">Unbounded durable writer cache under keep-all</span>
<span class="conflict-meta"><span class="conflict-rel">with Durability</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
</div>

## Related policies

- [RESOURCE_LIMITS](resource-limits.md)
- [DESTINATION_ORDER](destination-order.md)
- [LIFESPAN](lifespan.md)
- [RELIABILITY](reliability.md)
- [DURABILITY](durability.md)

### Example

The HIST QoS can be used to control how much robot data is retained. If a control station must preserve all robot positions since startup, kind=keep all can be set so that both the Publisher and the Subscriber store every sample. In contrast, for real-time tracking where only the latest position matters, setting kind=keep last with depth=1 ensures that each robot's Publisher retains and transmits only the most recent position, while the Subscriber receives only that single value.
