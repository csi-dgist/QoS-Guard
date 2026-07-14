# OWNERSHIP

Controls whether multiple Publishers can concurrently update the same instance, or, if not, which Publisher's value should be accepted.

## What it controls

Shared vs exclusive ownership of data instances across multiple Writers, and (with OWNERSHIP_STRENGTH) which Writer wins in exclusive mode.

* **SHARED**: Allows multiple Publishers to freely update the same instance.
* **EXCLUSIVE**: Only a single Publisher is allowed to update an instance, and its updates alone are delivered to Subscribers.<br>Priority is determined by the value of the OWNST STRENGTH QoS policy.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `kind` | SHARED | Can **not** be changed at runtime |

## Compatibility role

Matching and delivery — Writer and Reader ownership kind must match (RxO). Governs failover timing with deadline and liveliness, and dispose behavior with writer data lifecycle.

## When this conflicts

Ownership takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-04/">
<span class="conflict-title">Best-effort delivery with exclusive ownership</span>
<span class="conflict-meta"><span class="conflict-rel">with Reliability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-10/">
<span class="conflict-title">Exclusive ownership with no deadline to trigger failover</span>
<span class="conflict-meta"><span class="conflict-rel">with Deadline</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-11/">
<span class="conflict-title">Exclusive ownership with no liveliness lease to detect a dead writer</span>
<span class="conflict-meta"><span class="conflict-rel">with Liveliness</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-16/">
<span class="conflict-title">Auto-dispose on unregister races exclusive-ownership failover</span>
<span class="conflict-meta"><span class="conflict-rel">with Writer Data Lifecycle</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-26/">
<span class="conflict-title">Publisher and subscriber disagree on ownership kind</span>
<span class="conflict-meta"><span class="conflict-rel">with itself</span><span class="rule-consequence rule-consequence-connect">Won't connect</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-38/">
<span class="conflict-title">Deadline too tight for the link forces needless ownership handover</span>
<span class="conflict-meta"><span class="conflict-rel">with Deadline</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-39/">
<span class="conflict-title">Liveliness lease too tight for the link causes ownership flapping</span>
<span class="conflict-meta"><span class="conflict-rel">with Liveliness</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
</div>

## Related policies

- [RELIABILITY](reliability.md)
- [DEADLINE](deadline.md)
- [LIVELINESS](liveliness.md)
- [WRITER_DATA_LIFECYCLE](writer-data-lifecycle.md)
- [DESTINATION_ORDER](destination-order.md)

### Example

The OWNERSHIP and OWNERSHIP STRENGTH QoS policies can be used to manage shared resources or mission states consistently. For example, if multiple robots scan the environment simultaneously to build a shared map, shared mode allows the server to receive updates from all robots and generate a unified map. In contrast, for tasks that must be performed by a single robot, exclusive mode can be used with appropriate value assigned to each Publisher, ensuring that the active robot becomes the instance owner. If the current owner robot fails to respond due to a fault, DDS automatically transfers ownership to the standby robot with the next highest value, allowing the task to continue without interruption.
