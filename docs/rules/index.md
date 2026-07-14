# QoS Conflict Rules

Every rule below is one QoS setting combination that breaks or silently degrades ROS 2 communication, and QoS Guard checks your publishers and subscribers against all 40 of them. Search or filter to find the ones that match your setup, then open a rule to see a concrete example, how to fix it, and the evidence behind it.

Each rule leads to one of three outcomes.

<div class="consequence-legend">
<span class="rule-consequence rule-consequence-connect">Won't connect</span> the publisher and subscriber never match.
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span> they connect, but a QoS promise silently fails.
<span class="rule-consequence rule-consequence-resource">Wastes resources</span> they keep working, but memory, CPU, or timing degrades.
</div>

<div class="rules-toolbar" id="rule-filter-toolbar"></div>

<div class="rule-card-list" id="rule-card-list">

<article class="rulecard" id="rule-1" data-policies="History | Resource Limits" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-01/">History depth set deeper than the resource-limit cap</a>
<p class="rulecard-settings">If you set <b>History = KEEP_LAST with depth D</b> together with <b>Resource Limits max_samples_per_instance smaller than D</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">History &rarr; Resource Limits</span>
<a class="rulecard-ref" href="../evidence/rule-01/">Rule 1 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-2" data-policies="Resource Limits" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-02/">Total sample cap set below the per-instance cap</a>
<p class="rulecard-settings">If you set <b>Resource Limits max_samples = N</b> together with <b>max_samples_per_instance greater than N</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">Resource Limits &rarr; Resource Limits</span>
<a class="rulecard-ref" href="../evidence/rule-02/">Rule 2 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-3" data-policies="Reliability | Durability" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-03/">Best-effort delivery with transient-local durability</a>
<p class="rulecard-settings">If you set <b>Reliability = BEST_EFFORT</b> together with <b>Durability = TRANSIENT_LOCAL or stronger</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Reliability &rarr; Durability</span>
<a class="rulecard-ref" href="../evidence/rule-03/">Rule 3 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-4" data-policies="Reliability | Ownership" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-04/">Best-effort delivery with exclusive ownership</a>
<p class="rulecard-settings">If you set <b>Reliability = BEST_EFFORT</b> together with <b>Ownership = EXCLUSIVE</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Reliability &rarr; Ownership</span>
<a class="rulecard-ref" href="../evidence/rule-04/">Rule 4 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-5" data-policies="Reliability | Liveliness" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-05/">Best-effort delivery with manual liveliness</a>
<p class="rulecard-settings">If you set <b>Reliability = BEST_EFFORT</b> together with <b>Liveliness = MANUAL_BY_TOPIC</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Reliability &rarr; Liveliness</span>
<a class="rulecard-ref" href="../evidence/rule-05/">Rule 5 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-6" data-policies="Lifespan | Durability" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-06/">Finite lifespan expires the history that durability should replay</a>
<p class="rulecard-settings">If you set <b>Durability = TRANSIENT_LOCAL or stronger</b> together with <b>Lifespan set to a finite duration</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Lifespan &rarr; Durability</span>
<a class="rulecard-ref" href="../evidence/rule-06/">Rule 6 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-7" data-policies="Lifespan | Deadline" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-07/">Lifespan shorter than the deadline period</a>
<p class="rulecard-settings">If you set <b>Lifespan duration L</b> together with <b>Deadline period longer than L</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Lifespan &rarr; Deadline</span>
<a class="rulecard-ref" href="../evidence/rule-07/">Rule 7 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-8" data-policies="History | Destination Order" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-08/">Source-timestamp ordering with a history depth of one</a>
<p class="rulecard-settings">If you set <b>Destination Order = BY_SOURCE_TIMESTAMP</b> together with <b>History = KEEP_LAST with depth 1</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">History &rarr; Destination Order</span>
<a class="rulecard-ref" href="../evidence/rule-08/">Rule 8 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-9" data-policies="Resource Limits | Destination Order" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-09/">Source-timestamp ordering with a per-instance cap of one</a>
<p class="rulecard-settings">If you set <b>Destination Order = BY_SOURCE_TIMESTAMP with KEEP_ALL</b> together with <b>Resource Limits max_samples_per_instance = 1</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Resource Limits &rarr; Destination Order</span>
<a class="rulecard-ref" href="../evidence/rule-09/">Rule 9 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-10" data-policies="Deadline | Ownership" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-10/">Exclusive ownership with no deadline to trigger failover</a>
<p class="rulecard-settings">If you set <b>Ownership = EXCLUSIVE</b> together with <b>Deadline period = infinite</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Deadline &rarr; Ownership</span>
<a class="rulecard-ref" href="../evidence/rule-10/">Rule 10 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-11" data-policies="Liveliness | Ownership" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-11/">Exclusive ownership with no liveliness lease to detect a dead writer</a>
<p class="rulecard-settings">If you set <b>Ownership = EXCLUSIVE</b> together with <b>Liveliness lease_duration = infinite</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Liveliness &rarr; Ownership</span>
<a class="rulecard-ref" href="../evidence/rule-11/">Rule 11 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-12" data-policies="Liveliness | Reader Data Lifecycle" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-12/">No-writer purge that never fires because liveliness never expires</a>
<p class="rulecard-settings">If you set <b>Reader autopurge_no_writer_samples_delay is finite</b> together with <b>Liveliness lease_duration = infinite</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Liveliness &rarr; Reader Data Lifecycle</span>
<a class="rulecard-ref" href="../evidence/rule-12/">Rule 12 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-13" data-policies="Reader Data Lifecycle | Durability" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-13/">Disposed-sample purge that discards data durability should keep</a>
<p class="rulecard-settings">If you set <b>Durability = TRANSIENT or stronger</b> together with <b>Reader autopurge_disposed_samples_delay is finite</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Reader Data Lifecycle &rarr; Durability</span>
<a class="rulecard-ref" href="../evidence/rule-13/">Rule 13 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-14" data-policies="Partition | Deadline" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-14/">Deadline monitoring interrupted when partitions change</a>
<p class="rulecard-settings">If you set <b>Deadline period is finite</b> together with <b>Partition names are set (non-empty)</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Partition &rarr; Deadline</span>
<a class="rulecard-ref" href="../evidence/rule-14/">Rule 14 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-15" data-policies="Partition | Liveliness" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-15/">Manual liveliness disrupted when partitions change</a>
<p class="rulecard-settings">If you set <b>Liveliness = MANUAL_BY_TOPIC</b> together with <b>Partition names are set (non-empty)</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Partition &rarr; Liveliness</span>
<a class="rulecard-ref" href="../evidence/rule-15/">Rule 15 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-16" data-policies="Ownership | Writer Data Lifecycle" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-16/">Auto-dispose on unregister races exclusive-ownership failover</a>
<p class="rulecard-settings">If you set <b>Ownership = EXCLUSIVE</b> together with <b>Writer autodispose_unregistered_instances = true</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Ownership &rarr; Writer Data Lifecycle</span>
<a class="rulecard-ref" href="../evidence/rule-16/">Rule 16 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-17" data-policies="History | Lifespan" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-17/">Lifespan longer than the history window can ever hold</a>
<p class="rulecard-settings">If you set <b>History = KEEP_LAST with depth D</b> together with <b>Lifespan longer than D publish periods</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">History &rarr; Lifespan</span>
<a class="rulecard-ref" href="../evidence/rule-17/">Rule 17 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-18" data-policies="Resource Limits | Lifespan" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-18/">Lifespan longer than the resource-limit buffer can ever hold</a>
<p class="rulecard-settings">If you set <b>History = KEEP_ALL bounded by max_samples_per_instance M</b> together with <b>Lifespan longer than M publish periods</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">Resource Limits &rarr; Lifespan</span>
<a class="rulecard-ref" href="../evidence/rule-18/">Rule 18 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-19" data-policies="Entity Factory | Durability" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-19/">Manual enable with volatile durability drops pre-enable data</a>
<p class="rulecard-settings">If you set <b>Entity Factory autoenable_created_entities = false</b> together with <b>Durability = VOLATILE</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">Entity Factory &rarr; Durability</span>
<a class="rulecard-ref" href="../evidence/rule-19/">Rule 19 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-20" data-policies="Partition | Durability" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-20/">Partitions filter which retained samples a late joiner replays</a>
<p class="rulecard-settings">If you set <b>Durability = TRANSIENT_LOCAL or stronger</b> together with <b>Partition names are set (non-empty)</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">Partition &rarr; Durability</span>
<a class="rulecard-ref" href="../evidence/rule-20/">Rule 20 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-21" data-policies="Partition" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-21/">Publisher and subscriber share no partition name</a>
<p class="rulecard-settings">If you set <b>a Writer partition set</b> together with <b>a Reader partition set that does not overlap it</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">Partition &rarr; Partition</span>
<a class="rulecard-ref" href="../evidence/rule-21/">Rule 21 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-22" data-policies="Reliability" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-22/">Subscriber requires reliable, publisher offers best-effort</a>
<p class="rulecard-settings">If you set <b>Reader Reliability = RELIABLE</b> together with <b>Writer Reliability = BEST_EFFORT</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">Reliability &rarr; Reliability</span>
<a class="rulecard-ref" href="../evidence/rule-22/">Rule 22 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-23" data-policies="Durability" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-23/">Subscriber requires more durability than the publisher offers</a>
<p class="rulecard-settings">If you set <b>Reader Durability = TRANSIENT_LOCAL or stronger</b> together with <b>Writer Durability = VOLATILE (weaker)</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">Durability &rarr; Durability</span>
<a class="rulecard-ref" href="../evidence/rule-23/">Rule 23 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-24" data-policies="Deadline" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-24/">Publisher's deadline is slower than the subscriber demands</a>
<p class="rulecard-settings">If you set <b>Writer Deadline period P</b> together with <b>Reader Deadline period smaller than P</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">Deadline &rarr; Deadline</span>
<a class="rulecard-ref" href="../evidence/rule-24/">Rule 24 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-25" data-policies="Liveliness" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-25/">Publisher's liveliness is weaker than the subscriber requires</a>
<p class="rulecard-settings">If you set <b>a Reader that requests a strong liveliness kind or short lease</b> together with <b>a Writer that offers a weaker kind or a longer lease</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">Liveliness &rarr; Liveliness</span>
<a class="rulecard-ref" href="../evidence/rule-25/">Rule 25 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-26" data-policies="Ownership" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-26/">Publisher and subscriber disagree on ownership kind</a>
<p class="rulecard-settings">If you set <b>a Writer Ownership kind</b> together with <b>a Reader Ownership kind that differs from it</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">Ownership &rarr; Ownership</span>
<a class="rulecard-ref" href="../evidence/rule-26/">Rule 26 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-27" data-policies="Destination Order" data-consequence="connect">
<a class="rulecard-title" href="../evidence/rule-27/">Subscriber requires source-timestamp ordering the publisher does not offer</a>
<p class="rulecard-settings">If you set <b>Reader Destination Order = BY_SOURCE_TIMESTAMP</b> together with <b>Writer Destination Order = BY_RECEPTION_TIMESTAMP (weaker)</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-connect">Won't connect</span>
<span class="rulecard-policies">Destination Order &rarr; Destination Order</span>
<a class="rulecard-ref" href="../evidence/rule-27/">Rule 27 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-28" data-policies="Writer Data Lifecycle | Reader Data Lifecycle" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-28/">Immediate no-writer purge with auto-dispose turned off</a>
<p class="rulecard-settings">If you set <b>Writer autodispose_unregistered_instances = false</b> together with <b>Reader autopurge_no_writer_samples_delay = 0</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Writer Data Lifecycle &rarr; Reader Data Lifecycle</span>
<a class="rulecard-ref" href="../evidence/rule-28/">Rule 28 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-29" data-policies="Writer Data Lifecycle | Reader Data Lifecycle" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-29/">Disposed-sample purge that can never fire</a>
<p class="rulecard-settings">If you set <b>Writer autodispose_unregistered_instances = false</b> together with <b>Reader autopurge_disposed_samples_delay is finite</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">Writer Data Lifecycle &rarr; Reader Data Lifecycle</span>
<a class="rulecard-ref" href="../evidence/rule-29/">Rule 29 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-30" data-policies="Writer Data Lifecycle | Reader Data Lifecycle" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-30/">Reader cache that is never reclaimed after writers leave</a>
<p class="rulecard-settings">If you set <b>Writer autodispose_unregistered_instances = false</b> together with <b>Reader autopurge_no_writer_samples_delay = infinite</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">Writer Data Lifecycle &rarr; Reader Data Lifecycle</span>
<a class="rulecard-ref" href="../evidence/rule-30/">Rule 30 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-31" data-policies="History | Reliability" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-31/">Reliable history too shallow for the round-trip</a>
<p class="rulecard-settings">If you set <b>Reliability = RELIABLE with KEEP_LAST depth D</b> together with <b>a depth smaller than one round-trip of samples</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">History &rarr; Reliability</span>
<a class="rulecard-ref" href="../evidence/rule-31/">Rule 31 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-32" data-policies="Resource Limits | Reliability" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-32/">Reliable buffer too small for the round-trip</a>
<p class="rulecard-settings">If you set <b>Reliability = RELIABLE with KEEP_ALL</b> together with <b>max_samples_per_instance smaller than one round-trip of samples</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Resource Limits &rarr; Reliability</span>
<a class="rulecard-ref" href="../evidence/rule-32/">Rule 32 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-33" data-policies="Lifespan | Reliability" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-33/">Lifespan expires samples before reliable retransmission finishes</a>
<p class="rulecard-settings">If you set <b>Reliability = RELIABLE</b> together with <b>Lifespan shorter than one publish period plus a round-trip</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Lifespan &rarr; Reliability</span>
<a class="rulecard-ref" href="../evidence/rule-33/">Rule 33 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-34" data-policies="Reliability | Writer Data Lifecycle" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-34/">Best-effort delivery drops the dispose notification</a>
<p class="rulecard-settings">If you set <b>Writer autodispose_unregistered_instances = true</b> together with <b>Reliability = BEST_EFFORT</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Reliability &rarr; Writer Data Lifecycle</span>
<a class="rulecard-ref" href="../evidence/rule-34/">Rule 34 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-35" data-policies="Reliability | Deadline" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-35/">Best-effort loss triggers false deadline-missed events</a>
<p class="rulecard-settings">If you set <b>Reliability = BEST_EFFORT</b> together with <b>Deadline period is finite</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Reliability &rarr; Deadline</span>
<a class="rulecard-ref" href="../evidence/rule-35/">Rule 35 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-36" data-policies="Liveliness | Deadline" data-consequence="guarantee">
<a class="rulecard-title" href="../evidence/rule-36/">Liveliness lease shorter than the deadline period</a>
<p class="rulecard-settings">If you set <b>Deadline period is finite</b> together with <b>Liveliness lease_duration shorter than the deadline</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-guarantee">Breaks a guarantee</span>
<span class="rulecard-policies">Liveliness &rarr; Deadline</span>
<a class="rulecard-ref" href="../evidence/rule-36/">Rule 36 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-37" data-policies="History | Durability" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-37/">Unbounded durable writer cache under keep-all</a>
<p class="rulecard-settings">If you set <b>Durability = TRANSIENT_LOCAL with History = KEEP_ALL</b> together with <b>large or default resource limits</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">History &rarr; Durability</span>
<a class="rulecard-ref" href="../evidence/rule-37/">Rule 37 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-38" data-policies="Deadline | Ownership" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-38/">Deadline too tight for the link forces needless ownership handover</a>
<p class="rulecard-settings">If you set <b>Ownership = EXCLUSIVE</b> together with <b>Deadline shorter than one publish period plus a round-trip</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">Deadline &rarr; Ownership</span>
<a class="rulecard-ref" href="../evidence/rule-38/">Rule 38 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-39" data-policies="Liveliness | Ownership" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-39/">Liveliness lease too tight for the link causes ownership flapping</a>
<p class="rulecard-settings">If you set <b>Ownership = EXCLUSIVE</b> together with <b>Liveliness lease shorter than one publish period plus a round-trip</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">Liveliness &rarr; Ownership</span>
<a class="rulecard-ref" href="../evidence/rule-39/">Rule 39 &middot; details</a>
</div>
</article>

<article class="rulecard" id="rule-40" data-policies="Durability | Deadline" data-consequence="resource">
<a class="rulecard-title" href="../evidence/rule-40/">Durable replay resets the deadline timer for a late joiner</a>
<p class="rulecard-settings">If you set <b>Durability = TRANSIENT_LOCAL or stronger</b> together with <b>Deadline period is finite</b>.</p>
<div class="rulecard-foot">
<span class="rule-consequence rule-consequence-resource">Wastes resources</span>
<span class="rulecard-policies">Durability &rarr; Deadline</span>
<a class="rulecard-ref" href="../evidence/rule-40/">Rule 40 &middot; details</a>
</div>
</article>

</div>
