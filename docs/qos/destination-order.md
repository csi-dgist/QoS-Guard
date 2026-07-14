# DESTINATION_ORDER

Controls how a Subscriber orders samples from multiple Publishers targeting the same instance.

## What it controls

Whether the Reader presents samples in reception order or in the order they were stamped at the source.

* **BY_RECEPTION_TIMESTAMP**: Orders samples by their reception time at the Subscriber.
* **BY_SOURCE_TIMESTAMP**: Uses the timestamp assigned by the Publisher at publication, preserving the original creation order regardless of delays.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `kind` | BY_RECEPTION_TIMESTAMP | Can **not** be changed at runtime |

## Compatibility role

Matching and delivery — Writer offered ordering must be at least as strong as Reader requested ordering (RxO). Interacts with history depth and resource limits when source ordering is requested.

## When this conflicts

Destination Order takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-08/">
<span class="conflict-title">Source-timestamp ordering with a history depth of one</span>
<span class="conflict-meta"><span class="conflict-rel">with History</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-09/">
<span class="conflict-title">Source-timestamp ordering with a per-instance cap of one</span>
<span class="conflict-meta"><span class="conflict-rel">with Resource Limits</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-27/">
<span class="conflict-title">Subscriber requires source-timestamp ordering the publisher does not offer</span>
<span class="conflict-meta"><span class="conflict-rel">with itself</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
</div>

## Related policies

- [HISTORY](history.md)
- [RESOURCE_LIMITS](resource-limits.md)

### Example

The DESTORD QoS policy can be used to maintain data consistency when multiple robots simultaneously update the same instance. By configuring by source timestamp, samples are ordered by their creation time regardless of network delays or arrival order, allowing all robots to share a consistent view of the map. In contrast, for real-time data such as current positions, where the latest received value is most important, using by reception timestamp enables the Subscriber to reflect the earliest arriving sample immediately.
