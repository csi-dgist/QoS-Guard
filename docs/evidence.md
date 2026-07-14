# Evidence

Every rule is backed by evidence. Each page below explains the conflict in plain language, gives a concrete example and a fix, and then shows exactly why the rule is flagged, whether that is the DDS specification text, the engine source code, or a controlled measurement.

<div class="evidence-index-grid">
<a class="evidence-index-card" href="rule-01/">
<span class="evidence-index-title">History depth set deeper than the resource-limit cap</span>
<span class="evidence-index-sub">History &rarr; Resource Limits</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-02/">
<span class="evidence-index-title">Total sample cap set below the per-instance cap</span>
<span class="evidence-index-sub">Resource Limits &rarr; Resource Limits</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-03/">
<span class="evidence-index-title">Best-effort delivery with transient-local durability</span>
<span class="evidence-index-sub">Reliability &rarr; Durability</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-04/">
<span class="evidence-index-title">Best-effort delivery with exclusive ownership</span>
<span class="evidence-index-sub">Reliability &rarr; Ownership</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-05/">
<span class="evidence-index-title">Best-effort delivery with manual liveliness</span>
<span class="evidence-index-sub">Reliability &rarr; Liveliness</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-06/">
<span class="evidence-index-title">Finite lifespan expires the history that durability should replay</span>
<span class="evidence-index-sub">Lifespan &rarr; Durability</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-07/">
<span class="evidence-index-title">Lifespan shorter than the deadline period</span>
<span class="evidence-index-sub">Lifespan &rarr; Deadline</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-08/">
<span class="evidence-index-title">Source-timestamp ordering with a history depth of one</span>
<span class="evidence-index-sub">History &rarr; Destination Order</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-09/">
<span class="evidence-index-title">Source-timestamp ordering with a per-instance cap of one</span>
<span class="evidence-index-sub">Resource Limits &rarr; Destination Order</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-10/">
<span class="evidence-index-title">Exclusive ownership with no deadline to trigger failover</span>
<span class="evidence-index-sub">Deadline &rarr; Ownership</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-11/">
<span class="evidence-index-title">Exclusive ownership with no liveliness lease to detect a dead writer</span>
<span class="evidence-index-sub">Liveliness &rarr; Ownership</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-12/">
<span class="evidence-index-title">No-writer purge that never fires because liveliness never expires</span>
<span class="evidence-index-sub">Liveliness &rarr; Reader Data Lifecycle</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-13/">
<span class="evidence-index-title">Disposed-sample purge that discards data durability should keep</span>
<span class="evidence-index-sub">Reader Data Lifecycle &rarr; Durability</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-14/">
<span class="evidence-index-title">Deadline monitoring interrupted when partitions change</span>
<span class="evidence-index-sub">Partition &rarr; Deadline</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-15/">
<span class="evidence-index-title">Manual liveliness disrupted when partitions change</span>
<span class="evidence-index-sub">Partition &rarr; Liveliness</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-16/">
<span class="evidence-index-title">Auto-dispose on unregister races exclusive-ownership failover</span>
<span class="evidence-index-sub">Ownership &rarr; Writer Data Lifecycle</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-17/">
<span class="evidence-index-title">Lifespan longer than the history window can ever hold</span>
<span class="evidence-index-sub">History &rarr; Lifespan</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-18/">
<span class="evidence-index-title">Lifespan longer than the resource-limit buffer can ever hold</span>
<span class="evidence-index-sub">Resource Limits &rarr; Lifespan</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-19/">
<span class="evidence-index-title">Manual enable with volatile durability drops pre-enable data</span>
<span class="evidence-index-sub">Entity Factory &rarr; Durability</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-20/">
<span class="evidence-index-title">Partitions filter which retained samples a late joiner replays</span>
<span class="evidence-index-sub">Partition &rarr; Durability</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-21/">
<span class="evidence-index-title">Publisher and subscriber share no partition name</span>
<span class="evidence-index-sub">Partition &rarr; Partition</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-22/">
<span class="evidence-index-title">Subscriber requires reliable, publisher offers best-effort</span>
<span class="evidence-index-sub">Reliability &rarr; Reliability</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-23/">
<span class="evidence-index-title">Subscriber requires more durability than the publisher offers</span>
<span class="evidence-index-sub">Durability &rarr; Durability</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-24/">
<span class="evidence-index-title">Publisher's deadline is slower than the subscriber demands</span>
<span class="evidence-index-sub">Deadline &rarr; Deadline</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-25/">
<span class="evidence-index-title">Publisher's liveliness is weaker than the subscriber requires</span>
<span class="evidence-index-sub">Liveliness &rarr; Liveliness</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-26/">
<span class="evidence-index-title">Publisher and subscriber disagree on ownership kind</span>
<span class="evidence-index-sub">Ownership &rarr; Ownership</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-27/">
<span class="evidence-index-title">Subscriber requires source-timestamp ordering the publisher does not offer</span>
<span class="evidence-index-sub">Destination Order &rarr; Destination Order</span>
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
</a>
<a class="evidence-index-card" href="rule-28/">
<span class="evidence-index-title">Immediate no-writer purge with auto-dispose turned off</span>
<span class="evidence-index-sub">Writer Data Lifecycle &rarr; Reader Data Lifecycle</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-29/">
<span class="evidence-index-title">Disposed-sample purge that can never fire</span>
<span class="evidence-index-sub">Writer Data Lifecycle &rarr; Reader Data Lifecycle</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-30/">
<span class="evidence-index-title">Reader cache that is never reclaimed after writers leave</span>
<span class="evidence-index-sub">Writer Data Lifecycle &rarr; Reader Data Lifecycle</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-31/">
<span class="evidence-index-title">Reliable history too shallow for the round-trip</span>
<span class="evidence-index-sub">History &rarr; Reliability</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-32/">
<span class="evidence-index-title">Reliable buffer too small for the round-trip</span>
<span class="evidence-index-sub">Resource Limits &rarr; Reliability</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-33/">
<span class="evidence-index-title">Lifespan expires samples before reliable retransmission finishes</span>
<span class="evidence-index-sub">Lifespan &rarr; Reliability</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-34/">
<span class="evidence-index-title">Best-effort delivery drops the dispose notification</span>
<span class="evidence-index-sub">Reliability &rarr; Writer Data Lifecycle</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-35/">
<span class="evidence-index-title">Best-effort loss triggers false deadline-missed events</span>
<span class="evidence-index-sub">Reliability &rarr; Deadline</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-36/">
<span class="evidence-index-title">Liveliness lease shorter than the deadline period</span>
<span class="evidence-index-sub">Liveliness &rarr; Deadline</span>
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
</a>
<a class="evidence-index-card" href="rule-37/">
<span class="evidence-index-title">Unbounded durable writer cache under keep-all</span>
<span class="evidence-index-sub">History &rarr; Durability</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-38/">
<span class="evidence-index-title">Deadline too tight for the link forces needless ownership handover</span>
<span class="evidence-index-sub">Deadline &rarr; Ownership</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-39/">
<span class="evidence-index-title">Liveliness lease too tight for the link causes ownership flapping</span>
<span class="evidence-index-sub">Liveliness &rarr; Ownership</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
<a class="evidence-index-card" href="rule-40/">
<span class="evidence-index-title">Durable replay resets the deadline timer for a late joiner</span>
<span class="evidence-index-sub">Durability &rarr; Deadline</span>
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
</a>
</div>

## Common experimental conditions

Controlled loopback measurements use the following shared setup unless a rule page states otherwise.

<div class="evidence-conditions-grid">
  <div><strong>Platform</strong><span>ROS 2 Jazzy, loopback <code>lo</code>, delay and loss via <code>tc netem</code></span></div>
  <div><strong>Engines</strong><span>Fast DDS 2.14.6, Cyclone DDS 0.10.5</span></div>
  <div><strong>Older-version check</strong><span>Fast DDS 2.6.11 and Cyclone DDS 0.10.5 for version-sensitive cases</span></div>
  <div><strong>Repetitions</strong><span>5 runs by default, runtime-event rules at least 3, seed <code>20260630</code></span></div>
  <div><strong>Message size</strong><span>1024 B, with a 1 MB fragmentation probe on the reliable-history rule</span></div>
  <div class="evidence-network-card">
    <strong>Network grid</strong>
    <span>RTT 1 to 500 ms, loss 0 to 20%, publish period 10 to 100 ms</span>
  </div>
</div>

<style>
.evidence-conditions-grid { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:10px; margin:12px 0 8px; }
.evidence-conditions-grid > div { border:1px solid #dbe3ee; border-radius:8px; background:#f8fafc; padding:10px 12px; }
.evidence-conditions-grid strong { display:block; color:#334155; font-size:12.5px; margin-bottom:3px; }
.evidence-conditions-grid span { display:block; color:#475569; font-size:12.5px; line-height:1.45; }
@media (max-width:760px) { .evidence-conditions-grid { grid-template-columns:1fr; } }
</style>
