# DDS QoS Policy Guide (16 Policies)

> Source: *Dependency Chain Analysis of ROS 2 DDS QoS Policies: From Lifecycle Tutorial to Static Verification*  
> DDS Publish–Subscribe communication is divided into three phases: **Discovery → Data Exchange → Disassociation**.


<style>
.req-container {
    margin: 20px 0;
    border: 1px solid #e2e8f0;
    border-radius: 8px;
    background-color: #ffffff;
    overflow: hidden;
}
.req-item {
    display: flex;
    align-items: center;
    padding: 12px 16px;
    border-bottom: 1px solid #e2e8f0;
}
.req-item:last-child {
    border-bottom: none;
}
.req-label {
    min-width: 100px;
    font-weight: 700;
    color: #334155;
    font-size: 13px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
}
.req-container.req-table .req-label {
    width: 220px;
    min-width: 220px;
    max-width: 220px;
    flex-shrink: 0;
    box-sizing: border-box;
}
.req-value {
    color: #334155;
    font-size: 14px;
    border-left: 2px solid #e2e8f0;
    padding-left: 16px;
    margin-left: 8px;
}

.dds-table .dds-header {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 16px;
    padding: 12px 16px;
    background: linear-gradient(180deg, #f8fafc 0%, #f1f5f9 100%);
    border-bottom: 2px solid #e2e8f0;
    font-weight: 700;
    font-size: 12px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: #475569;
    text-align: center;
}
.dds-table .dds-header span { text-align: center; }
.dds-table .dds-row {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 16px;
    align-items: center;
    padding: 12px 16px;
    border-bottom: 1px solid #e2e8f0;
    font-size: 14px;
    color: #334155;
}
.dds-table .dds-row:last-child { border-bottom: none; }
.dds-table .dds-row span { text-align: center; }
.dds-table .dds-row span:first-child { font-weight: 700; text-align: left; }

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
    border-color: #4E5EB4;
    background-color: #f6f8fa;
    transform: translateY(-2px);
    box-shadow: 0 4px 8px rgba(0,0,0,0.1);
}

.qos-num {
    font-family: 'Courier New', monospace;
    font-weight: bold;
    color: #4E5EB4;
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

.lifecycle-container {
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    line-height: 1.6;
    color: #333;
    max-width: 800px;
  }
  .phase-card {
    border-left: 5px solid #4b95dd;
    background-color: #f1f5f9;
    padding: 10px 20px;
    margin-bottom: 20px;
    border-radius: 0 8px 8px 0;
    box-shadow: 0 2px 4px rgba(0,0,0,0.05);
  }
  .phase-title {
    font-size: 20px;
    font-weight: bold;
    color: #2c3e50;
    margin-bottom: 8px;
    display: flex;
    align-items: center;
  }
  .phase-desc {
    font-size: 15px;
    color: #555;
    margin-left: 20px;
  }
  b { color: #4E5EB4; }


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



<div class="lifecycle-container">
<div class="phase-card">
  <div class="phase-title">1. Discovery Phase</div>
  <div class="phase-desc">
    Entities with the same topic are matched through <b>PDP/EDP protocols</b> and established after verifying <b>QoS compatibility</b>.
  </div>
</div>

<div class="phase-card" style="border-left-color: #01af4e;">
  <div class="phase-title">2. Data Exchange Phase</div>
  <div class="phase-desc">
    Matched pairs exchange <b>user data</b> and <b>control metatraffic</b> (HEARTBEAT/ACKNACK) to ensure reliability and timeliness.
  </div>
</div>

<div class="phase-card" style="border-left-color: #f17232;">
  <div class="phase-title">3. Disassociation Phase</div>
  <div class="phase-desc">
    Communication ends by <b>disposing instances</b> or removing GUIDs, followed by purging all history after a <b>timeout</b>.
  </div>
</div>

</div>



<div align="center">
  <img src="../images/lifecycle.jpg" width="500">
</div>


<div class="req-container">
    <div class="req-item" style="font-weight: bold; background-color: #f1f5f9;">
        <span class="req-label" style="flex: 2;">QoS Policy</span>
        <span class="req-value" style="flex: 1; text-align: center;">Discovery</span>
        <span class="req-value" style="flex: 1; text-align: center;">Data Exchange</span>
        <span class="req-value" style="flex: 1; text-align: center;">Disassociation</span>
    </div>
    <div class="req-item"><span class="req-label" style="flex: 2;">ENTITY_FACTORY</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item"><span class="req-label" style="flex: 2;">PARTITION</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item"><span class="req-label" style="flex: 2;">USER_DATA</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item"><span class="req-label" style="flex: 2;">GROUP_DATA</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item">
        <span class="req-label" style="flex: 2;">TOPIC_DATA</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item"><span class="req-label" style="flex: 2;">RELIABILITY</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
    </div>
    <div class="req-item">
        <span class="req-label" style="flex: 2;">DURABILITY</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
    </div><div class="req-item"><span class="req-label" style="flex: 2;">DEADLINE</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item">
        <span class="req-label" style="flex: 2;">LIVELINESS</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
    </div>
    <div class="req-item">
        <span class="req-label" style="flex: 2;">HISTORY</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item">
        <span class="req-label" style="flex: 2;">RESOURCE_LIMITS</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item"><span class="req-label" style="flex: 2;">LIFESPAN</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item">
        <span class="req-label" style="flex: 2;">OWNERSHIP</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
    </div>
    <div class="req-item"><span class="req-label" style="flex: 2;">DESTINATION_ORDER</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
    </div>
    <div class="req-item">
        <span class="req-label" style="flex: 2;">WRITER_DATA_LIFECYCLE</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
    </div>
    <div class="req-item">
        <span class="req-label" style="flex: 2;">READER_DATA_LIFECYCLE</span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;"></span>
        <span class="req-value" style="flex: 1; text-align: center;">O</span>
    </div>
</div>

<hr class="hr-grad-left">

## 1. ENTITY FACTORY (ENTFAC)

**Role:** Controls **when** newly created DDS entities begin participating in discovery.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (boolean, default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">TRUE</span>
    <span class="req-value">Child entities are enabled immediately upon creation and participate in discovery</span>
  </div>
  <div class="req-item">
    <span class="req-label">FALSE</span>
    <span class="req-value">Entities do not participate in discovery until the application explicitly calls <code>enable()</code></span>
  </div>
  <div class="req-item">
    <span class="req-label">Lifecycle</span>
    <span class="req-value">Determines when the Discovery phase starts</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime; changes apply only to entities <strong>created after</strong> the update</span>
  </div>
</div>

**Example:** To activate a navigation module’s Publishers/Subscribers only after local sensor calibration or localization is complete, set `autoenable_created_entities=false` and call `enable()` at the appropriate time.

<hr class="hr-dashed">

## 2. PARTITION (PART)

**Role:** Introduces **logical segmentation** within a single DDS domain so that only certain Publisher/Subscriber groups match.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>names</code> (array of strings, default: empty string)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Applies to</span>
    <span class="req-value">Both Publisher and Subscriber</span>
  </div>
  <div class="req-item">
    <span class="req-label">Matching</span>
    <span class="req-value">Publisher and Subscriber match only if they share <strong>at least one</strong> partition name</span>
  </div>
  <div class="req-item">
    <span class="req-label">Exchanged</span>
    <span class="req-value">During the SEDP stage of the Discovery phase</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime; changes trigger rematching via SEDP (existing connections are broken)</span>
  </div>
</div>

**Example:** Delivery and inventory robots may use the same domain and topics such as `status` and `command`. A central management system can subscribe with `names=delivery` or `names=inventory` to receive data only from the desired group.

<hr class="hr-dashed">

## 3. USER DATA (USRDATA)

**Role:** Attaches **application-specific metadata** to entities such as DomainParticipant, Publisher, and Subscriber.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>value</code> (arbitrary byte sequence, default: empty)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Purpose</span>
    <span class="req-value">DDS does not interpret it; the value is carried in built-in topic samples (discovery messages)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Propagation</span>
    <span class="req-value">DomainParticipant → SPDP; Publisher/Subscriber → SEDP</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime; changes are reflected in the next built-in topic sample</span>
  </div>
</div>

**Example:** Each robot can embed `robot_id=R12`, `token=ABCD123` in its participant so the server can validate the token during SPDP and admit only authorized robots. A LiDAR Publisher can include `sensor=LiDAR`, `fov=270` so subscribers can choose an appropriate filtering strategy before receiving samples.

<hr class="hr-dashed">

## 4. GROUP DATA (GRPDATA)

**Role:** Attaches **application-specific metadata** to Publisher and Subscriber entities. Structure and behavior are the same as USRDATA.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>value</code> (arbitrary byte sequence)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Propagation</span>
    <span class="req-value">SEDP phase</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed freely at runtime</span>
  </div>
</div>

**Example:** Similar to PART for separating delivery vs. inventory robots; PART is used by DDS for matching, while GRPDATA is **interpreted by the application** in discovery callbacks.

<hr class="hr-dashed">

## 5. TOPIC DATA (TOPDATA)

**Role:** Attaches application-specific metadata to the **topic** entity. Not used for RxO matching; serves as an auxiliary channel for application information.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Format/Behavior</span>
    <span class="req-value">Same as USRDATA and GRPDATA</span>
  </div>
  <div class="req-item">
    <span class="req-label">Propagation</span>
    <span class="req-value">SEDP phase</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed freely at runtime</span>
  </div>
</div>

**Example:** Embed `schema=2.1`, `frame=lidar` in the scan cloud topic’s TOPDATA so an inventory app can check schema compatibility during topic discovery and avoid parsing errors.

<hr class="hr-dashed">

## 6. RELIABILITY (RELIAB)

**Role:** Determines whether data is sent **reliably** (with retransmission) or **best effort**.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameters</span>
    <span class="req-value"><code>kind</code> (best_effort / reliable), <code>max_blocking_time</code> (reliable mode only)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Defaults</span>
    <span class="req-value">Publisher: reliable; Subscriber and topic: best_effort</span>
  </div>
  <div class="req-item">
    <span class="req-label">best_effort</span>
    <span class="req-value">No ACK wait or retransmission; data is sent as fast as possible</span>
  </div>
  <div class="req-item">
    <span class="req-label">reliable</span>
    <span class="req-value">All samples in the Publisher HistoryCache are delivered; ACK/NACK triggers retransmission; order is preserved</span>
  </div>
  <div class="req-item">
    <span class="req-label">max_blocking_time</span>
    <span class="req-value">In reliable mode, maximum time <code>write()</code>/<code>dispose()</code> may block due to delayed ACKs or buffer unavailability</span>
  </div>
</div>

**Lifecycle**

- **Discovery:** SEDP RxO check. Matching succeeds only if Publisher.kind ≥ Subscriber.kind (best_effort < reliable).
- **Data Exchange:** In reliable mode, HEARTBEAT and ACKNACK/NACK FRAG enable retransmission. In best_effort, no control meta-traffic.
- **Disassociation:** dispose/unregister samples follow the same reliability rules.

**Example:** Use `reliable` on both sides for emergency stop or task assignment. Use `best_effort` for high-frequency streams (e.g., LiDAR, camera) to avoid retransmission overhead.

<hr class="hr-dashed">

## 7. DURABILITY (DURABL)

**Role:** Defines how **late-joining Subscribers** can receive previously published samples.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameters</span>
    <span class="req-value"><code>kind</code> (volatile / transient_local / transient / persistent) </span>
  </div>
  <div class="req-item">
    <span class="req-label">Defaults</span>
    <span class="req-value">VOLATILE</span>
  </div>

  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Immutable after the entity is enabled</span>
  </div>
</div>

**Lifecycle**

- **Discovery:** Matching succeeds only if Publisher.kind ≥ Subscriber.kind (volatile < transient_local < transient < persistent).
- **Data Exchange:** A non-volatile Publisher retransmits retained samples from HistoryCache to late joiners.
- **Disassociation:** For transient/persistent, the durability service retains samples and state.

**Example:** For global maps or mission plans that must remain available to robots that join later (or after restarts), use `transient` (survives process restart) or `persistent` (survives system reboot).

<hr class="hr-dashed">

## 8. DEADLINE (DEADLN)

**Role:** Specifies the **maximum period** within which a new sample for a given data instance must be produced by the Publisher and received by the Subscriber. If exceeded, DDS raises alarms on both sides.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>period</code> (default: infinity)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Meaning</span>
    <span class="req-value">A new sample must be produced and received within this period</span>
  </div>
  <div class="req-item">
    <span class="req-label">Use case</span>
    <span class="req-value">Monitoring <strong>periodic updates</strong> (e.g., periodic sensor data)</span>
  </div>
</div>

**Lifecycle**

- **Discovery:** RxO matching succeeds only if Publisher.period ≤ Subscriber.period.
- **Data Exchange:** Period is monitored per instance. Missed deadline on Publisher or Subscriber raises a notification for recovery or alerting.

**Example:** A robot publishes position and battery every second; the monitoring Subscriber uses period=1 s. If a sample is not received within that interval, a deadline-miss alarm indicates communication or robot failure.

<hr class="hr-dashed">

## 9. LIVELINESS (LIVENS)

**Role:** Allows a Subscriber to determine whether the corresponding **Publisher is still active**.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameters</span>
    <span class="req-value"><code>kind</code>  **automatic** (default), **manual_by_participant**, **manual_by_topic** <code>lease_duration</code> </span>
  </div>
  <div class="req-item">
    <span class="req-label">Defaults</span>
    <span class="req-value">Automatic </span>
  </div>

  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Immutable after the entity is enabled</span>
  </div>
</div>


**Lifecycle**

- **Discovery:** Matching succeeds only if Publisher.kind ≥ Subscriber.kind and Publisher.lease_duration ≤ Subscriber.lease_duration.
- **Data Exchange:** Periodic liveliness assertions or `assert_liveliness()`. In manual_by_topic, the LivelinessFlag in HEARTBEAT is used.
- **Disassociation:** If no assertion is received within lease_duration, the Subscriber marks the Publisher as not alive. If all writers disappear, the instance state becomes *not alive no writers*.

**Example:** A central monitor configures each robot’s Publisher with `kind=automatic` and `lease_duration=5` seconds. If no signal is received within 5 seconds, the monitor can gray out the robot’s icon or issue a warning.

<hr class="hr-dashed">

## 10. HISTORY (HIST)

**Role:** Determines **how many samples** per instance the Publisher keeps for retransmission and the Subscriber keeps before delivery to the application.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameters</span>
    <span class="req-value"><code>kind</code>, <code>depth</code></span>
  </div>
  <div class="req-item">
    <span class="req-label">kind</span>
    <span class="req-value"><strong>keep_last</strong>: retain only the most recent samples per instance, up to <code>depth</code> (default depth=1) / <strong>keep_all</strong>: retain all samples per instance; <code>depth</code> is ignored</span>
  </div>
  <div class="req-item">
    <span class="req-label">Applies to</span>
    <span class="req-value">Topic, Publisher, Subscriber. <strong>Immutable</strong> (set only at creation)</span>
  </div>
</div>

**Lifecycle**

- **Data Exchange:** Publisher and Subscriber each maintain a HistoryCache; data exchange is the process of synchronizing these caches. `kind` and `depth` limit how many samples are retained.

**Example:** For a control station that must keep all robot positions since startup, use `keep_all`. For real-time tracking where only the latest position matters, use `keep_last` with `depth=1`.

<hr class="hr-dashed">

## 11. RESOURCE LIMITS (RESLIM)

**Role:** Sets **upper bounds** on the number of instances and samples that topic, Publisher, and Subscriber entities can manage.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameters</span>
    <span class="req-value">`max_samples`, `max_instances`, `max_samples_per_instance`</span>
  </div>
  <div class="req-item">
    <span class="req-label">Defaults</span>
    <span class="req-value">OMG standard: unlimited; implementations may impose defaults </span>
  </div>

  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Immutable after configuration</span>
  </div>
</div>

**Lifecycle**

- **Data Exchange:** Limits how many samples/instances can be stored in HistoryCache, preventing overflow and memory exhaustion.

**Example:** For real-time position only, set the Publisher’s `max_instances` low to save memory. On the Subscriber, set `max_samples_per_instance` high enough when many robot instances may be present.

<hr class="hr-dashed">

## 12. LIFESPAN (LFSPAN)

**Role:** Defines **how long** a sample published by a Publisher remains valid. After expiration, the sample is removed from both Publisher and Subscriber HistoryCache.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>duration</code> (default: infinity)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Applies to</span>
    <span class="req-value">Topic, Publisher (and possibly Subscriber, depending on implementation)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Behavior</span>
    <span class="req-value">Expiration time = publication time + duration; expired samples are automatically removed</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value"><strong>Mutable</strong>; can be changed after the entity is enabled</span>
  </div>
</div>

**Lifecycle**

- **Data Exchange:** Expiration is tracked per sample. Expired samples are no longer accessible to the Subscriber and help manage memory.

**Example:** For position or battery data where only the last few seconds matter, set `duration` accordingly so older samples are removed. For command logs that must be delivered over a long time, set `duration=infinity`.

<hr class="hr-dashed">

## 13. OWNERSHIP (+STRENGTH) (OWNST)

**Role:** Determines whether **multiple Publishers** can update the same instance, and if not, **which Publisher’s** value is accepted.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameters</span>
    <span class="req-value">shared, Exclusive</span>
  </div>
  <div class="req-item">
    <span class="req-label">Defaults</span>
    <span class="req-value">OMG standard: unlimited; implementations may impose defaults </span>
  </div>

  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Immutable after enable</span>
  </div>
</div>

**Lifecycle**

- **Discovery:** Matching succeeds only if Publisher.kind and Subscriber.kind are **identical**.
- **Data Exchange:** In shared mode, the Subscriber receives updates from all Publishers; in exclusive mode, only from the highest-strength Publisher.
- **Disassociation:** In exclusive mode, only the **current owner** can effectively issue `dispose()`/`unregister()`.

**Example:** Use `shared` when multiple robots update a shared map. For a task that must be performed by a single robot, use `exclusive` with strength so the active robot is the owner; on failure, ownership can transfer to the next-highest strength robot.

<hr class="hr-dashed">

## 14. DESTINATION ORDER (DESTORD)

**Role:** Determines **in what order** a Subscriber applies samples when **multiple Publishers** write to the same instance.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">kind</span>
    <span class="req-value"><strong>by_reception_timestamp</strong> (default): order by arrival time at the Subscriber / <strong>by_source_timestamp</strong>: order by timestamp assigned by the Publisher (preserves creation order)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value"><strong>Immutable</strong> after enable</span>
  </div>
</div>

**Lifecycle**

- **Discovery:** Matching succeeds only if Publisher.kind ≥ Subscriber.kind (by_reception_timestamp < by_source_timestamp).
- **Data Exchange:** The chosen kind governs the processing order of samples from multiple Publishers for the same instance.

**Example:** Use `by_source_timestamp` when multiple robots update the same map instance so all share a consistent view. For real-time position where “latest received” matters, use `by_reception_timestamp`.

<hr class="hr-dashed">

## 15. WRITER DATA LIFECYCLE (WDLIFE)

**Role:** Determines whether the Publisher **notifies Subscribers with dispose()** when it **unregisters** an instance.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autodispose_unregistered_instances</code> (default: true)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Applies to</span>
    <span class="req-value">Publisher only</span>
  </div>
  <div class="req-item">
    <span class="req-label">true</span>
    <span class="req-value">When <code>unregister()</code> is called, the instance is automatically marked disposed → Subscriber sees it as deleted</span>
  </div>
  <div class="req-item">
    <span class="req-label">false</span>
    <span class="req-value"><code>unregister()</code> only disassociates the writer from the instance; the application must explicitly call <code>dispose()</code> to delete it</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value"><strong>Mutable</strong></span>
  </div>
</div>

**Lifecycle**

- **Disassociation:** When the Publisher calls `unregister()` or is deleted, this setting determines whether the Subscriber treats the instance as *not alive disposed* or *not alive no writers*.

**Example:** When a robot publishes detected objects, `autodispose_unregistered_instances=true` removes the instance from the map as “processed” when the task is done. Set to `false` if another robot may rediscover and update the same object.

<hr class="hr-dashed">

## 16. READER DATA LIFECYCLE (RDLIFE)

**Role:** Determines **how long** a Subscriber retains samples for instances that are **disposed** or have **no associated Publishers** before purging them.

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameters</span>
    <span class="req-value"><code>autopurge_disposed_samples_delay</code>, <code>autopurge_no_writer_samples_delay</code></span>
  </div>
  <div class="req-item">
    <span class="req-label">Applies to</span>
    <span class="req-value">Subscriber only</span>
  </div>
  <div class="req-item">
    <span class="req-label">autopurge_disposed_samples_delay</span>
    <span class="req-value">Time to retain samples/metadata after the instance becomes <em>not alive disposed</em> (default: infinity)</span>
  </div>
  <div class="req-item">
    <span class="req-label">autopurge_no_writer_samples_delay</span>
    <span class="req-value">Time to retain after the instance becomes <em>not alive no writers</em> (default: infinity)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value"><strong>Mutable</strong></span>
  </div>
</div>

**Lifecycle**

- **Disassociation:** When the Publisher calls `dispose()`, the instance becomes *not alive disposed*. When all writers disappear, it becomes *not alive no writers*. After the corresponding delay, the Subscriber purges the instance and samples to reclaim memory.

**Example:** In a temporary storage area with fast-moving pallets, set `autopurge_disposed_samples_delay=0` to purge as soon as dispose() is received. For critical static objects, set `autopurge_no_writer_samples_delay=300` seconds so newly joined robots can still inspect after brief communication loss.

<hr class="hr-grad-left">

## Summary: Metadata, Matching, Cache, and Lifecycle

<div class="req-container dds-table">
  <div class="dds-header">
    <span>Category</span>
    <span>QoS</span>
    <span>One-line summary</span>
  </div>
  <div class="dds-row">
    <span>Discovery timing</span>
    <span>ENTFAC</span>
    <span>When entities participate in discovery</span>
  </div>
  <div class="dds-row">
    <span>Logical segmentation</span>
    <span>PART</span>
    <span>Partition names to separate or group topic flows</span>
  </div>
  <div class="dds-row">
    <span>Metadata</span>
    <span>USRDATA, GRPDATA, TOPDATA</span>
    <span>Application info on Participant / Pub-Sub / Topic</span>
  </div>
  <div class="dds-row">
    <span>Delivery guarantee</span>
    <span>RELIAB</span>
    <span>best_effort vs reliable (retransmission, ordering)</span>
  </div>
  <div class="dds-row">
    <span>Late joiners</span>
    <span>DURABL</span>
    <span>How much past data late joiners can receive</span>
  </div>
  <div class="dds-row">
    <span>Temporal constraints</span>
    <span>DEADLN, LIVENS</span>
    <span>Period (deadline) monitoring; Publisher liveness</span>
  </div>
  <div class="dds-row">
    <span>Cache</span>
    <span>HIST, RESLIM, LFSPAN</span>
    <span>How many samples, upper bounds, validity duration</span>
  </div>
  <div class="dds-row">
    <span>Multiple writers</span>
    <span>OWNST, DESTORD</span>
    <span>Single owner vs shared; order by reception vs source timestamp</span>
  </div>
  <div class="dds-row">
    <span>Instance cleanup</span>
    <span>WDLIFE, RDLIFE</span>
    <span>Dispose on unregister; when to purge disposed/no-writer samples</span>
  </div>
</div>

This document is based on the lifecycle-based QoS tutorial and mobile-robot examples (Appendix A) from the paper.
