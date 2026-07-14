# WRITER_DATA_LIFECYCLE

Controls whether a Publisher should notify Subscribers with a dispose() when it unregisters an instance.

## What it controls

Automatic dispose behavior when a Writer calls `unregister_instance()`.

* **TRUE**: An instance to be automatically marked as disposed when an unregister() is called, so that it is recognized as deleted on the Subscriber side.
* **FALSE**: The unregister() only disassociates the writer from the instance without disposing of it; the application must explicitly call dispose() to fully delete the instance.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `autodispose_unregistered_instances` | TRUE | Can be changed at runtime |

## Compatibility role

Lifecycle and disassociation — governs how unregister propagates to Readers. Interacts with ownership failover, reliability, and reader purge delays.

## When this conflicts

Writer Data Lifecycle takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-16/">
<span class="conflict-title">Auto-dispose on unregister races exclusive-ownership failover</span>
<span class="conflict-meta"><span class="conflict-rel">with Ownership</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-28/">
<span class="conflict-title">Immediate no-writer purge with auto-dispose turned off</span>
<span class="conflict-meta"><span class="conflict-rel">with Reader Data Lifecycle</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-29/">
<span class="conflict-title">Disposed-sample purge that can never fire</span>
<span class="conflict-meta"><span class="conflict-rel">with Reader Data Lifecycle</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-30/">
<span class="conflict-title">Reader cache that is never reclaimed after writers leave</span>
<span class="conflict-meta"><span class="conflict-rel">with Reader Data Lifecycle</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-34/">
<span class="conflict-title">Best-effort delivery drops the dispose notification</span>
<span class="conflict-meta"><span class="conflict-rel">with Reliability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
</div>

## Related policies

- [OWNERSHIP](ownership.md)
- [READER_DATA_LIFECYCLE](reader-data-lifecycle.md)
- [RELIABILITY](reliability.md)

### Example

The WDLIFE QoS can be used to explicitly manage the lifecycle of object-based tasks. For example, when a robot detects an object with its sensors, it can publish the object's key, position, type, and status on a topic, allowing other robots or the control system to subscribe and build a shared environment model. When the task is completed, the post processing behavior depends on the detecting robot's autodispose unregistered instances setting. If true, the instance is immediately disposed of and removed from the map or marked as "processed"; if false, only the Publisher-instance link is removed while the object information remains active, allowing another robot to rediscover and update the same object, thereby improving collaboration flexibility.
