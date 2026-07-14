# RELIABILITY

Determines whether entities such as topic, Publisher, and Subscriber transmit data reliably.

## What it controls

Whether samples are delivered with acknowledgment and retransmission, or on a best-effort basis without guaranteed delivery.

* **RELIABLE**: Attempts to deliver all samples stored in the Publisher HistoryCache to the Subscriber.<br>Retransmissions occur upon request using ACK/NACK signaling. <br>(max blocking time limits how long a Publisher's write() or dispose() operation can be blocked due to delayed ACKs or buffer unavailability.)
* **BEST_EFFORT**: Publisher does not wait for ACKs and does not retransmit lost samples.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `kind` | Publisher-reliable, Subscriber-best_effort | Can **not** be changed at runtime |
| `max_blocking_time` | — | Can **not** be changed at runtime |

## Compatibility role

Matching and delivery — Writer offered reliability must be at least as strong as Reader requested reliability (RxO). Governs retransmission, dispose delivery, and interaction with liveliness, durability, and timing policies.

## When this conflicts

Reliability takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-03/">
<span class="conflict-title">Best-effort delivery with transient-local durability</span>
<span class="conflict-meta"><span class="conflict-rel">with Durability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-04/">
<span class="conflict-title">Best-effort delivery with exclusive ownership</span>
<span class="conflict-meta"><span class="conflict-rel">with Ownership</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-05/">
<span class="conflict-title">Best-effort delivery with manual liveliness</span>
<span class="conflict-meta"><span class="conflict-rel">with Liveliness</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-22/">
<span class="conflict-title">Subscriber requires reliable, publisher offers best-effort</span>
<span class="conflict-meta"><span class="conflict-rel">with itself</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-31/">
<span class="conflict-title">Reliable history too shallow for the round-trip</span>
<span class="conflict-meta"><span class="conflict-rel">with History</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-32/">
<span class="conflict-title">Reliable buffer too small for the round-trip</span>
<span class="conflict-meta"><span class="conflict-rel">with Resource Limits</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-33/">
<span class="conflict-title">Lifespan expires samples before reliable retransmission finishes</span>
<span class="conflict-meta"><span class="conflict-rel">with Lifespan</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-34/">
<span class="conflict-title">Best-effort delivery drops the dispose notification</span>
<span class="conflict-meta"><span class="conflict-rel">with Writer Data Lifecycle</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-35/">
<span class="conflict-title">Best-effort loss triggers false deadline-missed events</span>
<span class="conflict-meta"><span class="conflict-rel">with Deadline</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
</div>

## Related policies

- [DURABILITY](durability.md)
- [OWNERSHIP](ownership.md)
- [LIVELINESS](liveliness.md)
- [HISTORY](history.md)
- [RESOURCE_LIMITS](resource-limits.md)
- [LIFESPAN](lifespan.md)
- [DEADLINE](deadline.md)
- [WRITER_DATA_LIFECYCLE](writer-data-lifecycle.md)

### Example

The RELIAB QoS can be used to balance safety and efficiency by configuring topics according to their criticality. For commands such as emergency stops or task assignments, both Publisher and Subscriber should use reliable, ensuring guaranteed delivery. In contrast, high-frequency data such as LiDAR scans or camera streams can use best effort, which avoids retransmission overhead and tolerates occasional loss.
