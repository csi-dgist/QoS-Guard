# RESOURCE_LIMITS

Defines upper bounds on the number of instances and samples that the topic, Publisher, and Subscriber entities can manage.

## What it controls

Maximum samples, instances, and samples-per-instance that an entity can hold in its caches.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `max_samples` | — | Can **not** be changed at runtime |
| `max_instances` | — | Can **not** be changed at runtime |
| `max_samples_per_instance` | — | Can **not** be changed at runtime |

## Compatibility role

Cache bounds — caps memory use on Topic, Writer, and Reader. Must be consistent with history depth and interacts with lifespan, reliability, and destination order.

## When this conflicts

Resource Limits takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-01/">
<span class="conflict-title">History depth set deeper than the resource-limit cap</span>
<span class="conflict-meta"><span class="conflict-rel">with History</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-02/">
<span class="conflict-title">Total sample cap set below the per-instance cap</span>
<span class="conflict-meta"><span class="conflict-rel">with itself</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-09/">
<span class="conflict-title">Source-timestamp ordering with a per-instance cap of one</span>
<span class="conflict-meta"><span class="conflict-rel">with Destination Order</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-18/">
<span class="conflict-title">Lifespan longer than the resource-limit buffer can ever hold</span>
<span class="conflict-meta"><span class="conflict-rel">with Lifespan</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-32/">
<span class="conflict-title">Reliable buffer too small for the round-trip</span>
<span class="conflict-meta"><span class="conflict-rel">with Reliability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
</div>

## Related policies

- [HISTORY](history.md)
- [DESTINATION_ORDER](destination-order.md)
- [LIFESPAN](lifespan.md)
- [RELIABILITY](reliability.md)

### Example

The RESLIM QoS can be used to manage resources and maintain communication stability. For instance, where only the most recent information is important and historical records are less critical, such as a robot's real-time position, the Publisher's max instances can be set to a small value to avoid excessive memory usage. On the Subscriber side, where the number of participating robots may be large or variable, the max samples per instance value should be set high enough to keep minimal data for each robot instance.
