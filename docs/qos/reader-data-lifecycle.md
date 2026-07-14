# READER_DATA_LIFECYCLE

Controls how long a Subscriber retains samples that have been disposed of or are no longer associated with any Publisher.

## What it controls

Delays before the Reader automatically purges disposed samples or samples with no active Writer.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `autopurge_disposed_samples_delay` | infinity | Can be changed at runtime |
| `autopurge_nowriter_samples_delay` | infinity | Can be changed at runtime |

## Compatibility role

Lifecycle and disassociation — governs how long disposed or orphaned instances remain in the Reader cache. Interacts with liveliness, durability, and writer autodispose settings.

## When this conflicts

Reader Data Lifecycle takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-12/">
<span class="conflict-title">No-writer purge that never fires because liveliness never expires</span>
<span class="conflict-meta"><span class="conflict-rel">with Liveliness</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-13/">
<span class="conflict-title">Disposed-sample purge that discards data durability should keep</span>
<span class="conflict-meta"><span class="conflict-rel">with Durability</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-28/">
<span class="conflict-title">Immediate no-writer purge with auto-dispose turned off</span>
<span class="conflict-meta"><span class="conflict-rel">with Writer Data Lifecycle</span><span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-29/">
<span class="conflict-title">Disposed-sample purge that can never fire</span>
<span class="conflict-meta"><span class="conflict-rel">with Writer Data Lifecycle</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
<a class="conflict-item" href="../../evidence/rule-30/">
<span class="conflict-title">Reader cache that is never reclaimed after writers leave</span>
<span class="conflict-meta"><span class="conflict-rel">with Writer Data Lifecycle</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
</div>

## Related policies

- [LIVELINESS](liveliness.md)
- [DURABILITY](durability.md)
- [WRITER_DATA_LIFECYCLE](writer-data-lifecycle.md)

### Example

The RDLIFE QoS policy enables tiered instance management for improved efficiency. For example, in a temporary storage area where hundreds of pallets move quickly and historical positions become obsolete immediately, setting autopurge disposed samples delay to 0 seconds ensures that the cache is cleared as soon as a robot calls dispose(). This keeps the cache lightweight and prevents unnecessary memory growth. In contrast, for static, critical objects awaiting inspection after relocation, setting autopurge no writer samples delay to 300 seconds allows newly joined robots to continue inspection even after brief communication interruptions. By adjusting the reader-side delay values based on context, essential data can be preserved while irrelevant information is promptly discarded, ensuring efficient use of system resources.
