# DEADLINE

Specifies the maximum interval within which a new sample for a given data instance must be produced by the Publisher and received by the Subscriber.

## What it controls

The maximum expected period between consecutive samples per instance. Missed deadlines trigger `requested_deadline_missed` or `offered_deadline_missed` callbacks.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `period` | infinity | Can be changed at runtime |

## Compatibility role

Matching and timing — Writer offered deadline period must not exceed Reader requested period (RxO). Monitors sample freshness during data exchange and interacts with lifespan and liveliness lease durations.

## When this conflicts

Deadline takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-07/">
<span class="conflict-title">Lifespan shorter than the deadline period</span>
<span class="conflict-meta"><span class="conflict-rel">with Lifespan</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-10/">
<span class="conflict-title">Exclusive ownership with no deadline to trigger failover</span>
<span class="conflict-meta"><span class="conflict-rel">with Ownership</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-14/">
<span class="conflict-title">Deadline monitoring interrupted when partitions change</span>
<span class="conflict-meta"><span class="conflict-rel">with Partition</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-24/">
<span class="conflict-title">Publisher's deadline is slower than the subscriber demands</span>
<span class="conflict-meta"><span class="conflict-rel">with itself</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-35/">
<span class="conflict-title">Best-effort loss triggers false deadline-missed events</span>
<span class="conflict-meta"><span class="conflict-rel">with Reliability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-36/">
<span class="conflict-title">Liveliness lease shorter than the deadline period</span>
<span class="conflict-meta"><span class="conflict-rel">with Liveliness</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-38/">
<span class="conflict-title">Deadline too tight for the link forces needless ownership handover</span>
<span class="conflict-meta"><span class="conflict-rel">with Ownership</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-40/">
<span class="conflict-title">Durable replay resets the deadline timer for a late joiner</span>
<span class="conflict-meta"><span class="conflict-rel">with Durability</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
</div>

## Related policies

- [LIFESPAN](lifespan.md)
- [OWNERSHIP](ownership.md)
- [PARTITION](partition.md)
- [RELIABILITY](reliability.md)
- [LIVELINESS](liveliness.md)
- [DURABILITY](durability.md)

### Example

The DEADLN QoS can be used for real-time monitoring of robot status. Each robot may publish its position and battery level through ROS 2 topics every second. The Publisher and the central monitoring system's Subscriber are both configured with a period of 1 second. If a new sample fails to arrive within this interval, the monitoring system receives a deadline-miss notification, enabling immediate detection of communication failures or faults in that data stream. The application can then respond by issuing alerts or stopping the robot, thereby enhancing overall system safety and reliability.
