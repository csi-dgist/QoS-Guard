# LIVELINESS

Determines whether its corresponding Publisher is still active.

## What it controls

How and when a Writer asserts that it is alive, and how long a Reader waits before reporting `liveliness_lost`.

* **AUTOMATIC**: The DomainParticipant asserts liveliness implicitly by periodical liveliness assertions.
* **MANUAL_BY_PARTICIPANT**: A single assertion from any entity within a DomainParticipant marks all of its Publishers as alive.
* **MANUAL_BY_TOPIC**: Each Publisher must explicitly assert its own liveliness by publishing HEARTBEAT samples or calling assert liveliness().

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `kind` | AUTOMATIC | Can **not** be changed at runtime |
| `lease_duration` | infinite | Can **not** be changed at runtime |

## Compatibility role

Matching and timing — Writer offered liveliness kind and lease must satisfy Reader requirements (RxO). Distinguishes publisher process health from per-sample deadline monitoring.

## When this conflicts

Liveliness takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-05/">
<span class="conflict-title">Best-effort delivery with manual liveliness</span>
<span class="conflict-meta"><span class="conflict-rel">with Reliability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-11/">
<span class="conflict-title">Exclusive ownership with no liveliness lease to detect a dead writer</span>
<span class="conflict-meta"><span class="conflict-rel">with Ownership</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-12/">
<span class="conflict-title">No-writer purge that never fires because liveliness never expires</span>
<span class="conflict-meta"><span class="conflict-rel">with Reader Data Lifecycle</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-15/">
<span class="conflict-title">Manual liveliness disrupted when partitions change</span>
<span class="conflict-meta"><span class="conflict-rel">with Partition</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-25/">
<span class="conflict-title">Publisher's liveliness is weaker than the subscriber requires</span>
<span class="conflict-meta"><span class="conflict-rel">with itself</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-36/">
<span class="conflict-title">Liveliness lease shorter than the deadline period</span>
<span class="conflict-meta"><span class="conflict-rel">with Deadline</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-39/">
<span class="conflict-title">Liveliness lease too tight for the link causes ownership flapping</span>
<span class="conflict-meta"><span class="conflict-rel">with Ownership</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
</div>

## Related policies

- [RELIABILITY](reliability.md)
- [OWNERSHIP](ownership.md)
- [PARTITION](partition.md)
- [DEADLINE](deadline.md)
- [READER_DATA_LIFECYCLE](reader-data-lifecycle.md)

### Example

The LIVENS QoS can be used to verify whether the publishing process itself is still active, whereas the DEADLN QoS ensures the timely delivery of individual data samples. This policy enables a central monitoring system to automatically track the operational status of each robot. For instance, each robot may publish position and battery level via a Publisher. The central Subscriber is configured with kind set to automatic and lease duration set to 5 seconds, causing DDS to refresh liveliness every five seconds. If a signal is not received within the lease duration, a liveliness notification is triggered to indicate that the robot is inactive. The monitoring application can then respond by graying out the robot's icon on the map or issuing a warning.
