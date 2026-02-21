# DDS QoS Policy Guide (16 Policies)

> Source: *Dependency Chain Analysis of ROS 2 DDS QoS Policies: From Lifecycle Tutorial to Static Verification*  
> DDS Publish–Subscribe communication is divided into three phases: **Discovery → Data Exchange → Disassociation**.


<style>
/* qos.md 전용 커스텀 스타일 */
.qos-toc-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
    gap: 15px;
    margin-top: 20px;
    font-family: 'Segoe UI', Roboto, sans-serif;
}

.qos-card {
    background: #ffffff;
    border: 1px solid #e1e4e8;
    border-radius: 8px;
    padding: 12px 16px;
    text-decoration: none !important;
    color: #24292e !important;
    transition: all 0.2s ease-in-out;
    display: flex;
    align-items: center;
    box-shadow: 0 2px 4px rgba(0,0,0,0.04);
}

.qos-card:hover {
    border-color: #0366d6;
    background-color: #f6f8fa;
    transform: translateY(-2px);
    box-shadow: 0 4px 8px rgba(0,0,0,0.1);
}

.qos-num {
    font-family: 'Courier New', monospace;
    font-weight: bold;
    color: #0366d6;
    margin-right: 12px;
    font-size: 1.1em;
}

.qos-title {
    font-weight: 500;
    font-size: 0.95em;
    line-height: 1.2;
}

.qos-abbr {
    display: block;
    font-size: 0.75em;
    color: #6a737d;
    margin-top: 2px;
}  
</style>

<hr class="hr-grad-left"> 

## Table of Contents

<div class="qos-toc-grid">
  <a href="#1-entity-factory-entfac" class="qos-card"><span class="qos-num">01</span><div class="qos-title">ENTITY FACTORY<span class="qos-abbr">ENTFAC</span></div></a>
  <a href="#2-partition-part" class="qos-card"><span class="qos-num">02</span><div class="qos-title">PARTITION<span class="qos-abbr">PART</span></div></a>
  <a href="#3-user-data-usrdata" class="qos-card"><span class="qos-num">03</span><div class="qos-title">USER DATA<span class="qos-abbr">USRDATA</span></div></a>
  <a href="#4-group-data-grpdata" class="qos-card"><span class="qos-num">04</span><div class="qos-title">GROUP DATA<span class="qos-abbr">GRPDATA</span></div></a>
  <a href="#5-topic-data-topdata" class="qos-card"><span class="qos-num">05</span><div class="qos-title">TOPIC DATA<span class="qos-abbr">TOPDATA</span></div></a>
  <a href="#6-reliability-reliab" class="qos-card"><span class="qos-num">06</span><div class="qos-title">RELIABILITY<span class="qos-abbr">RELIAB</span></div></a>
  <a href="#7-durability-durabl" class="qos-card"><span class="qos-num">07</span><div class="qos-title">DURABILITY<span class="qos-abbr">DURABL</span></div></a>
  <a href="#8-deadline-deadln" class="qos-card"><span class="qos-num">08</span><div class="qos-title">DEADLINE<span class="qos-abbr">DEADLN</span></div></a>
  <a href="#9-liveliness-livens" class="qos-card"><span class="qos-num">09</span><div class="qos-title">LIVELINESS<span class="qos-abbr">LIVENS</span></div></a>
  <a href="#10-history-hist" class="qos-card"><span class="qos-num">10</span><div class="qos-title">HISTORY<span class="qos-abbr">HIST</span></div></a>
  <a href="#11-resource-limits-reslim" class="qos-card"><span class="qos-num">11</span><div class="qos-title">RESOURCE LIMITS<span class="qos-abbr">RESLIM</span></div></a>
  <a href="#12-lifespan-lfspan" class="qos-card"><span class="qos-num">12</span><div class="qos-title">LIFESPAN<span class="qos-abbr">LFSPAN</span></div></a>
  <a href="#13-ownership-strength-ownst" class="qos-card"><span class="qos-num">13</span><div class="qos-title">OWNERSHIP<span class="qos-abbr">OWNST</span></div></a>
  <a href="#14-destination-order-destord" class="qos-card"><span class="qos-num">14</span><div class="qos-title">DESTINATION ORDER<span class="qos-abbr">DESTORD</span></div></a>
  <a href="#15-writer-data-lifecycle-wdlife" class="qos-card"><span class="qos-num">15</span><div class="qos-title">WRITER DATA LIFECYCLE<span class="qos-abbr">WDLIFE</span></div></a>
  <a href="#16-reader-data-lifecycle-rdlife" class="qos-card"><span class="qos-num">16</span><div class="qos-title">READER DATA LIFECYCLE<span class="qos-abbr">RDLIFE</span></div></a>
</div>

<hr class="hr-grad-left">

## QoS Mapping by Lifecycle Phase

| Phase | Applicable QoS |
|-------|----------------|
| **Discovery** | ENTFAC, PART, USRDATA, GRPDATA, TOPDATA |
| **Data Exchange** | RELIAB, DURABL, DEADLN, LIVENS, HIST, RESLIM, LFSPAN, OWNST, DESTORD |
| **Disassociation** | RELIAB, DURABL, LIVENS, OWNST, WDLIFE, RDLIFE |

<hr class="hr-grad-left">

## 1. ENTITY FACTORY (ENTFAC)

**Role:** Controls **when** newly created DDS entities begin participating in discovery.

| Item | Description |
|------|-------------|
| **Parameter** | `autoenable_created_entities` (boolean, default: TRUE) |
| **TRUE** | Child entities are enabled immediately upon creation and participate in discovery |
| **FALSE** | Entities do not participate in discovery until the application explicitly calls `enable()` |
| **Lifecycle** | Determines when the Discovery phase starts |
| **Mutability** | Can be changed at runtime; changes apply only to entities **created after** the update |

**Example:** To activate a navigation module’s Publishers/Subscribers only after local sensor calibration or localization is complete, set `autoenable_created_entities=false` and call `enable()` at the appropriate time.

<hr class="hr-dashed">

## 2. PARTITION (PART)

**Role:** Introduces **logical segmentation** within a single DDS domain so that only certain Publisher/Subscriber groups match.

| Item | Description |
|------|-------------|
| **Parameter** | `names` (array of strings, default: empty string) |
| **Applies to** | Both Publisher and Subscriber |
| **Matching** | Publisher and Subscriber match only if they share **at least one** partition name |
| **Exchanged** | During the SEDP stage of the Discovery phase |
| **Mutability** | Can be changed at runtime; changes trigger rematching via SEDP (existing connections are broken) |

**Example:** Delivery and inventory robots may use the same domain and topics such as `status` and `command`. A central management system can subscribe with `names=delivery` or `names=inventory` to receive data only from the desired group.

<hr class="hr-dashed">

## 3. USER DATA (USRDATA)

**Role:** Attaches **application-specific metadata** to entities such as DomainParticipant, Publisher, and Subscriber.

| Item | Description |
|------|-------------|
| **Parameter** | `value` (arbitrary byte sequence, default: empty) |
| **Purpose** | DDS does not interpret it; the value is carried in built-in topic samples (discovery messages) |
| **Propagation** | DomainParticipant → SPDP; Publisher/Subscriber → SEDP |
| **Mutability** | Can be changed at runtime; changes are reflected in the next built-in topic sample |

**Example:** Each robot can embed `robot_id=R12`, `token=ABCD123` in its participant so the server can validate the token during SPDP and admit only authorized robots. A LiDAR Publisher can include `sensor=LiDAR`, `fov=270` so subscribers can choose an appropriate filtering strategy before receiving samples.

<hr class="hr-dashed">

## 4. GROUP DATA (GRPDATA)

**Role:** Attaches **application-specific metadata** to Publisher and Subscriber entities. Structure and behavior are the same as USRDATA.

| Item | Description |
|------|-------------|
| **Parameter** | `value` (arbitrary byte sequence) |
| **Propagation** | SEDP phase |
| **Mutability** | Can be changed freely at runtime |

**Example:** Similar to PART for separating delivery vs. inventory robots; PART is used by DDS for matching, while GRPDATA is **interpreted by the application** in discovery callbacks.

<hr class="hr-dashed">

## 5. TOPIC DATA (TOPDATA)

**Role:** Attaches application-specific metadata to the **topic** entity. Not used for RxO matching; serves as an auxiliary channel for application information.

| Item | Description |
|------|-------------|
| **Format/Behavior** | Same as USRDATA and GRPDATA |
| **Propagation** | SEDP phase |
| **Mutability** | Can be changed freely at runtime |

**Example:** Embed `schema=2.1`, `frame=lidar` in the scan cloud topic’s TOPDATA so an inventory app can check schema compatibility during topic discovery and avoid parsing errors.

<hr class="hr-dashed">

## 6. RELIABILITY (RELIAB)

**Role:** Determines whether data is sent **reliably** (with retransmission) or **best effort**.

| Item | Description |
|------|-------------|
| **Parameters** | `kind` (best_effort / reliable), `max_blocking_time` (reliable mode only) |
| **Defaults** | Publisher: reliable; Subscriber and topic: best_effort |
| **best_effort** | No ACK wait or retransmission; data is sent as fast as possible |
| **reliable** | All samples in the Publisher HistoryCache are delivered; ACK/NACK triggers retransmission; order is preserved |
| **max_blocking_time** | In reliable mode, maximum time `write()`/`dispose()` may block due to delayed ACKs or buffer unavailability |

**Lifecycle**

- **Discovery:** SEDP RxO check. Matching succeeds only if Publisher.kind ≥ Subscriber.kind (best_effort < reliable).
- **Data Exchange:** In reliable mode, HEARTBEAT and ACKNACK/NACK FRAG enable retransmission. In best_effort, no control meta-traffic.
- **Disassociation:** dispose/unregister samples follow the same reliability rules.

**Example:** Use `reliable` on both sides for emergency stop or task assignment. Use `best_effort` for high-frequency streams (e.g., LiDAR, camera) to avoid retransmission overhead.

<hr class="hr-dashed">

## 7. DURABILITY (DURABL)

**Role:** Defines how **late-joining Subscribers** can receive previously published samples.

| Item | Description |
|------|-------------|
| **Parameter** | `kind` (volatile / transient_local / transient / persistent) |
| **Default** | volatile |
| **volatile** | No delivery of past samples |
| **transient_local** | Samples retained in the Publisher’s HistoryCache while it is active; delivered to late joiners |
| **transient** | Data retained after the Publisher ends, in **volatile memory** |
| **persistent** | Data retained in **non-volatile storage** (e.g., files, database) |
| **Mutability** | **Immutable** after the entity is enabled |

**Lifecycle**

- **Discovery:** Matching succeeds only if Publisher.kind ≥ Subscriber.kind (volatile < transient_local < transient < persistent).
- **Data Exchange:** A non-volatile Publisher retransmits retained samples from HistoryCache to late joiners.
- **Disassociation:** For transient/persistent, the durability service retains samples and state.

**Example:** For global maps or mission plans that must remain available to robots that join later (or after restarts), use `transient` (survives process restart) or `persistent` (survives system reboot).

<hr class="hr-dashed">

## 8. DEADLINE (DEADLN)

**Role:** Specifies the **maximum period** within which a new sample for a given data instance must be produced by the Publisher and received by the Subscriber. If exceeded, DDS raises alarms on both sides.

| Item | Description |
|------|-------------|
| **Parameter** | `period` (default: infinity) |
| **Meaning** | A new sample must be produced and received within this period |
| **Use case** | Monitoring **periodic updates** (e.g., periodic sensor data) |

**Lifecycle**

- **Discovery:** RxO matching succeeds only if Publisher.period ≤ Subscriber.period.
- **Data Exchange:** Period is monitored per instance. Missed deadline on Publisher or Subscriber raises a notification for recovery or alerting.

**Example:** A robot publishes position and battery every second; the monitoring Subscriber uses period=1 s. If a sample is not received within that interval, a deadline-miss alarm indicates communication or robot failure.

<hr class="hr-dashed">

## 9. LIVELINESS (LIVENS)

**Role:** Allows a Subscriber to determine whether the corresponding **Publisher is still active**.

| Item | Description |
|------|-------------|
| **Parameters** | `kind`, `lease_duration` |
| **kind** | Who asserts liveliness: **automatic** (default), **manual_by_participant**, **manual_by_topic** |
| **lease_duration** | Maximum time the Subscriber waits after missing liveliness assertions before declaring the Publisher “not alive” (default: infinite) |
| **manual_by_topic** | Each Publisher asserts via HEARTBEAT or `assert_liveliness()` |

**Lifecycle**

- **Discovery:** Matching succeeds only if Publisher.kind ≥ Subscriber.kind and Publisher.lease_duration ≤ Subscriber.lease_duration.
- **Data Exchange:** Periodic liveliness assertions or `assert_liveliness()`. In manual_by_topic, the LivelinessFlag in HEARTBEAT is used.
- **Disassociation:** If no assertion is received within lease_duration, the Subscriber marks the Publisher as not alive. If all writers disappear, the instance state becomes *not alive no writers*.

**Example:** A central monitor configures each robot’s Publisher with `kind=automatic` and `lease_duration=5` seconds. If no signal is received within 5 seconds, the monitor can gray out the robot’s icon or issue a warning.

<hr class="hr-dashed">

## 10. HISTORY (HIST)

**Role:** Determines **how many samples** per instance the Publisher keeps for retransmission and the Subscriber keeps before delivery to the application.

| Item | Description |
|------|-------------|
| **Parameters** | `kind`, `depth` |
| **kind** | **keep_last**: retain only the most recent samples per instance, up to `depth` (default depth=1) / **keep_all**: retain all samples per instance; `depth` is ignored |
| **Applies to** | Topic, Publisher, Subscriber. **Immutable** (set only at creation) |

**Lifecycle**

- **Data Exchange:** Publisher and Subscriber each maintain a HistoryCache; data exchange is the process of synchronizing these caches. `kind` and `depth` limit how many samples are retained.

**Example:** For a control station that must keep all robot positions since startup, use `keep_all`. For real-time tracking where only the latest position matters, use `keep_last` with `depth=1`.

<hr class="hr-dashed">

## 11. RESOURCE LIMITS (RESLIM)

**Role:** Sets **upper bounds** on the number of instances and samples that topic, Publisher, and Subscriber entities can manage.

| Item | Description |
|------|-------------|
| **Parameters** | `max_samples`, `max_instances`, `max_samples_per_instance` |
| **Instance** | A data object identified by the topic type’s key |
| **Sample** | A transmission unit containing an instance’s data and metadata |
| **Defaults** | OMG standard: unlimited; implementations may impose defaults |
| **Mutability** | **Immutable** after configuration |

**Lifecycle**

- **Data Exchange:** Limits how many samples/instances can be stored in HistoryCache, preventing overflow and memory exhaustion.

**Example:** For real-time position only, set the Publisher’s `max_instances` low to save memory. On the Subscriber, set `max_samples_per_instance` high enough when many robot instances may be present.

<hr class="hr-dashed">

## 12. LIFESPAN (LFSPAN)

**Role:** Defines **how long** a sample published by a Publisher remains valid. After expiration, the sample is removed from both Publisher and Subscriber HistoryCache.

| Item | Description |
|------|-------------|
| **Parameter** | `duration` (default: infinity) |
| **Applies to** | Topic, Publisher (and possibly Subscriber, depending on implementation) |
| **Behavior** | Expiration time = publication time + duration; expired samples are automatically removed |
| **Mutability** | **Mutable**; can be changed after the entity is enabled |

**Lifecycle**

- **Data Exchange:** Expiration is tracked per sample. Expired samples are no longer accessible to the Subscriber and help manage memory.

**Example:** For position or battery data where only the last few seconds matter, set `duration` accordingly so older samples are removed. For command logs that must be delivered over a long time, set `duration=infinity`.

<hr class="hr-dashed">

## 13. OWNERSHIP (+STRENGTH) (OWNST)

**Role:** Determines whether **multiple Publishers** can update the same instance, and if not, **which Publisher’s** value is accepted.

| Item | Description |
|------|-------------|
| **kind** | **shared** (default): multiple Publishers can update concurrently / **exclusive**: only one Publisher per instance; only its updates are delivered to Subscribers |
| **OWNERSHIP_STRENGTH** | Meaningful only in exclusive mode. Each Publisher has an integer strength (default 0). The highest strength is the “owner.” |
| **Mutability** | kind is **immutable** after enable; strength is **mutable** |

**Lifecycle**

- **Discovery:** Matching succeeds only if Publisher.kind and Subscriber.kind are **identical**.
- **Data Exchange:** In shared mode, the Subscriber receives updates from all Publishers; in exclusive mode, only from the highest-strength Publisher.
- **Disassociation:** In exclusive mode, only the **current owner** can effectively issue `dispose()`/`unregister()`.

**Example:** Use `shared` when multiple robots update a shared map. For a task that must be performed by a single robot, use `exclusive` with strength so the active robot is the owner; on failure, ownership can transfer to the next-highest strength robot.

<hr class="hr-dashed">

## 14. DESTINATION ORDER (DESTORD)

**Role:** Determines **in what order** a Subscriber applies samples when **multiple Publishers** write to the same instance.

| Item | Description |
|------|-------------|
| **kind** | **by_reception_timestamp** (default): order by arrival time at the Subscriber / **by_source_timestamp**: order by timestamp assigned by the Publisher (preserves creation order) |
| **Mutability** | **Immutable** after enable |

**Lifecycle**

- **Discovery:** Matching succeeds only if Publisher.kind ≥ Subscriber.kind (by_reception_timestamp < by_source_timestamp).
- **Data Exchange:** The chosen kind governs the processing order of samples from multiple Publishers for the same instance.

**Example:** Use `by_source_timestamp` when multiple robots update the same map instance so all share a consistent view. For real-time position where “latest received” matters, use `by_reception_timestamp`.

<hr class="hr-dashed">

## 15. WRITER DATA LIFECYCLE (WDLIFE)

**Role:** Determines whether the Publisher **notifies Subscribers with dispose()** when it **unregisters** an instance.

| Item | Description |
|------|-------------|
| **Parameter** | `autodispose_unregistered_instances` (default: true) |
| **Applies to** | Publisher only |
| **true** | When `unregister()` is called, the instance is automatically marked disposed → Subscriber sees it as deleted |
| **false** | `unregister()` only disassociates the writer from the instance; the application must explicitly call `dispose()` to delete it |
| **Mutability** | **Mutable** |

**Lifecycle**

- **Disassociation:** When the Publisher calls `unregister()` or is deleted, this setting determines whether the Subscriber treats the instance as *not alive disposed* or *not alive no writers*.

**Example:** When a robot publishes detected objects, `autodispose_unregistered_instances=true` removes the instance from the map as “processed” when the task is done. Set to `false` if another robot may rediscover and update the same object.

<hr class="hr-dashed">

## 16. READER DATA LIFECYCLE (RDLIFE)

**Role:** Determines **how long** a Subscriber retains samples for instances that are **disposed** or have **no associated Publishers** before purging them.

| Item | Description |
|------|-------------|
| **Parameters** | `autopurge_disposed_samples_delay`, `autopurge_no_writer_samples_delay` |
| **Applies to** | Subscriber only |
| **autopurge_disposed_samples_delay** | Time to retain samples/metadata after the instance becomes *not alive disposed* (default: infinity) |
| **autopurge_no_writer_samples_delay** | Time to retain after the instance becomes *not alive no writers* (default: infinity) |
| **Mutability** | **Mutable** |

**Lifecycle**

- **Disassociation:** When the Publisher calls `dispose()`, the instance becomes *not alive disposed*. When all writers disappear, it becomes *not alive no writers*. After the corresponding delay, the Subscriber purges the instance and samples to reclaim memory.

**Example:** In a temporary storage area with fast-moving pallets, set `autopurge_disposed_samples_delay=0` to purge as soon as dispose() is received. For critical static objects, set `autopurge_no_writer_samples_delay=300` seconds so newly joined robots can still inspect after brief communication loss.

<hr class="hr-grad-left">

## Summary: Metadata, Matching, Cache, and Lifecycle

| Category | QoS | One-line summary |
|----------|-----|------------------|
| **Discovery timing** | ENTFAC | When entities participate in discovery |
| **Logical segmentation** | PART | Partition names to separate or group topic flows |
| **Metadata** | USRDATA, GRPDATA, TOPDATA | Application info on Participant / Pub-Sub / Topic |
| **Delivery guarantee** | RELIAB | best_effort vs reliable (retransmission, ordering) |
| **Late joiners** | DURABL | How much past data late joiners can receive |
| **Temporal constraints** | DEADLN, LIVENS | Period (deadline) monitoring; Publisher liveness |
| **Cache** | HIST, RESLIM, LFSPAN | How many samples, upper bounds, validity duration |
| **Multiple writers** | OWNST, DESTORD | Single owner vs shared; order by reception vs source timestamp |
| **Instance cleanup** | WDLIFE, RDLIFE | Dispose on unregister; when to purge disposed/no-writer samples |

This document is based on the lifecycle-based QoS tutorial and mobile-robot examples (Appendix A) from the paper.
