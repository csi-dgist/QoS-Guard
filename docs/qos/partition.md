# PARTITION

Introduces logical segmentation within a single DDS domain.

## What it controls

Which logical partition names a Publisher or Subscriber belongs to. Endpoints match only when their partition name sets overlap.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `names` | empty string | Can be changed at runtime |

## Compatibility role

Matching — partition names must overlap between Writer and Reader for a match. Affects discovery and late-join behavior when combined with durability or timing policies.

## When this conflicts

Partition takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-14/">
<span class="conflict-title">Deadline monitoring interrupted when partitions change</span>
<span class="conflict-meta"><span class="conflict-rel">with Deadline</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-15/">
<span class="conflict-title">Manual liveliness disrupted when partitions change</span>
<span class="conflict-meta"><span class="conflict-rel">with Liveliness</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-20/">
<span class="conflict-title">Partitions filter which retained samples a late joiner replays</span>
<span class="conflict-meta"><span class="conflict-rel">with Durability</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-21/">
<span class="conflict-title">Publisher and subscriber share no partition name</span>
<span class="conflict-meta"><span class="conflict-rel">with itself</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
</div>

## Related policies

- [DEADLINE](deadline.md)
- [LIVELINESS](liveliness.md)
- [DURABILITY](durability.md)

### Example

The PART QoS can be used to separate identical data types into multiple logical groups without the need to define additional topics or create new domains. For instance, delivery robots and inventory robots may share common topics such as "status" and "command" within the same domain, but still require distinct data flows. By setting the names=delivery for the delivery robots and names=inventory for the inventory robots, a central management system can subscribe only to the desired partition and selectively receive data from a specific group of robots.
