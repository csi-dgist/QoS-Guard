# DDS QoS Policy Guide

> A comprehensive reference for configuring 16 key DDS QoS policies

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
    background-color: #ffffff;
    padding: 10px 20px;
    margin-bottom: 20px;
    border-radius: 0 8px 8px 0;
    box-shadow: 0 2px 6px rgba(0,0,0,0.05),
        2px 0 6px rgba(0,0,0,0.05),
        0px -2px 6px rgba(0,0,0,0.05);
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
  b { color: #394a5b; }

</style>

<hr class="hr-grad-left"> 

## 16 QoS Policies

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

## Lifecycle of DDS Communication

<div align="center">
  <img src="../images/lifecycle.jpg" width="700">
    <figcaption style="font-style: italic; color: #666; margin-top: 10px;">
    Three Phases: Discovery → Data Exchange → Disassociation.
  </figcaption>
</div>

<div class="lifecycle-container">
<div class="phase-card">
  <div class="phase-title">1. Discovery Phase</div>
  <div class="phase-desc">
    Entities with the same topic are matched through <b>PDP/EDP protocols</b> and <br>established after verifying <b>QoS compatibility</b>.
  </div>
</div>

<div class="phase-card" style="border-left-color: #01af4e;">
  <div class="phase-title">2. Data Exchange Phase</div>
  <div class="phase-desc">
    Matched pairs exchange <b>user data</b> and <b>control metatraffic</b> (HEARTBEAT/ACKNACK) to <br>ensure reliability and timeliness.
  </div>
</div>

<div class="phase-card" style="border-left-color: #f17232;">
  <div class="phase-title">3. Disassociation Phase</div>
  <div class="phase-desc">
    Communication ends by <b>disposing instances</b> or removing GUIDs, followed by <br>purging all history after a <b>timeout</b>.
  </div>
</div>
</div>

<details>
  <summary>Lifecycle Mapping</summary>
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
</details>

<hr class="hr-grad-left">

## 1. ENTITY FACTORY (ENTFAC)

> **Controls whether newly created DDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC QoS can be used to conserve resources and allow multiple robots to initiate discovery simultaneously under synchronized conditions. For instance, the Publishers and Subscribers of a navigation module may be activated only after completing local sensor calibration or localization. By setting autoenable created entities=false, the system delays communication until the robot is ready by explicitly calling enable() at the appropriate time

<hr class="hr-dashed">

## 2. PARTITION (PART)

> **Introduces logical segmentation with in a single DDS domain**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>names</code> (default: emptystring)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Example
The PART QoS can be used to separate identical data types into multiple logical groups without the need to define additional topics or create new domains. For instance, delivery robots and inventory robots may share common topics such as “status” and “command” within the same domain, but still require distinct data flows. By setting the names=delivery for the delivery robots and names=inventory for the inventory robots, a central management system can subscribe only to the desired partition and selectively receive data from a specific group of robots.

<hr class="hr-dashed">

## 3. USER DATA (USRDATA)

> **Allows application-specific meta data to be attached to DDS entities such as Domain Participant, Publisher, and Subscriber**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>value</code> (default: empty sequence)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Example
The USRDATA QoS can be used to flexibly deliver iden tity, authentication, and configuration information without re quiring additional topics or separate domains. For example, each robot may embed value such as robot id=R12 and to ken=ABCD123 in its participant, allowing the server to inspect the token during the SPDP phase and admit only authorized robots while blocking others. Similarly, a Publisher for a LiDAR topic may include value such as sensor=LiDAR and fov=270 in its USRDATA, enabling the subscribing application to determine sensor configuration and immediately select an appropriate filtering strategy before receiving any samples.

<hr class="hr-dashed">

## 4. GROUP DATA (GRPDATA)

> **Attaches application-specific metadata to the Publisher and Subscriber entities**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>value</code> (default: empty sequence)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Example
The GRPDATA QoS can be used to logically segment data f lows in a manner similar to the PART QoS. For example, if delivery and inventory robots share topics within the same domain, assigning value=delivery or value=inventory to each Publisher or Subscriber allows the central management server to read these values during discovery callbacks. Although similar to PART, the key distinction is that PART enforces matching at the DDS level, whereas GRPDATA leaves the interpretation of the field entirely to the application.

<hr class="hr-dashed">

## 5. TOPIC DATA (TOPDATA)

> **Attaches application-specific metadata to the topic entity**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>value</code> (default: empty sequence)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Example
The TOPDATA QoS can be used by applications to verify schema compatibility in advance. For example, each robot can embed value such as schema=2.1 and frame=lidar in the TOPDATA of the scan cloud topic. During topic discovery, an inventory management application can read this information, and if the schema is incompatible, it can prevent subscription and avoid data parsing errors.

<hr class="hr-dashed">

## 6. RELIABILITY (RELIAB)

> **Determines whether entities such as topic, Publisher, and Subscriber transmit data reliably**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter1</span>
    <span class="req-value"><code>kind</code> (default: Publisher-reliable, Subscriber-best_effrot)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Parameter2</span>
    <span class="req-value"><code>max blocking time</code></span>
  </div>  
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can not be changed at runtime</span>
  </div>
</div>

### kind Mode
* **RELIABLE**: Attempts to deliver all samples stored in the Publisher HistoryCache to the Subscriber.<br>Retransmissions occur upon request using ACK/NACK signaling. <br>(max blocking time limits how long a Publisher’s write() or dispose() operation can be blocked due to delayed ACKs or buffer unavailability.)
* **BEST_EFFORT**: Publisher does not wait for ACKs and does not retransmit lost samples.

### Example
The RELIAB QoS can be used to balance safety and efficiency by configuring topics according to their criticality. For commands such as emergency stops or task assignments, both Publisher and Subscriber should use reliable, ensuring guaranteed delivery. In contrast, high-frequency data such as LiDAR scans or camera streams can use best effort, which avoids retransmission overhead and tolerates occasional loss. 

<hr class="hr-dashed">

## 7. DURABILITY (DURABL)

> **Determines how a late-joining Subscriber can receive previously published samples from a Publisher**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>kind</code> (default: VOLATILE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can <b>not</b> be changed at runtime</span>
  </div>
</div>

### kind Mode
* **VOLATILE**: Does not send any previous samples to newly joined Subscribers.
* **TRANSIENT_LOCAL**: Retains samples in the Publisher’s HistoryCache while it is active, allowing late-joining Subscribers to access previously published data.
* **TRANSIENT**: Preserve data after a Publisher has been terminated. Retains data in volatile memory.
* **PERSISTENT**: Preserve data after a Publisher has been terminated. Uses non-volatile storage such as files or databases

### Example
The DURABL QoS can be used to allow newly added or recovered robots to immediately access the same information, thereby improving both system robustness and collaborative efficiency. Data such as global maps or mission plans, whose availability must not depend on the lifespan of a specific robot, should remain accessible to robots that join after the Publisher has terminated or rebooted. In such cases, DURABL should be set to transient if data must persist throughout process restarts, or to persistent if it must survive system-wide reboots.

<hr class="hr-dashed">

## 8. DEADLINE (DEADLN)

> **Specifies the maximum interval within which a new sample for a given data instance must be produced by the Publisher and
received by the Subscriber**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>period</code> (default: infinity)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Example
The DEADLN QoS can be used for real-time monitoring of robot status. Each robot may publish its position and battery level through ROS 2 topics every second. The Publisher and the central monitoring system’s Subscriber are both configured with a period of 1 second. If a new sample fails to arrive within this interval, the monitoring system receives a deadline-miss notification, enabling immediate detection of communication failures or faults in that data stream. The application can then respond by issuing alerts or stopping the robot, thereby enhancing overall system safety and reliability.

<hr class="hr-dashed">

## 9. LIVELINESS (LIVENS)

> **Determine whether its corresponding Publisher is still active**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter1</span>
    <span class="req-value"><code>kind</code> (default: AUTOMATIC)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Parameter2</span>
    <span class="req-value"><code>lease_duration (default: infinite)</code></span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can not be changed at runtime</span>
  </div>
</div>

### kind Mode
* **AUTOMATIC**: The DomainParticipant asserts liveliness implicitly by periodical liveliness assertions.
* **MANUAL_BY_PARTICIPANT**: A single assertion from any entity within a DomainParticipant marks all of its Publishers as alive.
* **MANUAL_BY_TOPIC**: Each Publisher must explicitly assert its own liveliness by publishing HEARTBEAT samples or calling assert liveliness().

### Example
The LIVENS QoS can be used to verify whether the pub lishing process itself is still active, whereas the DEADLN QoS ensures the timely delivery of individual data samples. This policy enables a central monitoring system to automatically track the operational status of each robot. For instance, each robot may publish position and battery level via a Publisher. The central Subscriber is configured with kind set to automatic and lease duration set to 5 seconds, causing DDS to refresh liveliness every five seconds. If a signal is not received within the lease duration, a liveliness notification is triggered to indicate that the robot is inactive. The monitoring application can then respond by graying out the robot’s icon on the map or issuing a warning.

<hr class="hr-dashed">

## 10. HISTORY (HIST)

> **Determines how many samples a Publisher retains in its HistoryCache for retransmission, and how many samples a Subscriber stores before they are delivered to the application.**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>kind</code> (default: KEEP_LAST, depth=1)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can not be changed at runtime</span>
  </div>
</div>

### kind Mode
* **KEEP_LAST**: Retains only the most recent samples per instance, with depth specifying the maximum number of samples to keep.
* **KEEP_ALL**: Stores all samples for each instance and attempts to deliver as many as possible.

### Example
The HIST QoS can be used to control how much robot data is retained. If a control station must preserve all robot positions since startup, kind=keep all can be set so that both the Publisher and the Subscriber store every sample. In contrast, for real-time tracking where only the latest position matters, setting kind=keep last with depth=1 ensures that each robot’s Publisher retains and transmits only the most recent position, while the Subscriber receives only that single value.

<hr class="hr-dashed">

## 11. RESOURCE LIMITS (RESLIM)

> **Defines upper bounds on the number of instances and samples that the topic, Publishe, and Subscriber entities can manage**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter1</span>
    <span class="req-value"><code>max_samples</code></span>
  </div>
  <div class="req-item">
    <span class="req-label">Parameter2</span>
    <span class="req-value"><code>max_instances</code></span>
  </div>
  <div class="req-item">
    <span class="req-label">Parameter3</span>
    <span class="req-value"><code>max samples per instance</code></span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can not be changed at runtime</span>
  </div>
</div>

### Example
The RESLIM QoS can be used to manage resources and maintain communication stability. For instance, where only the most recent information is important and historical records are less critical, such as a robot’s real-time position, the Publisher’s max instances can be set to a small value to avoid excessive memory usage. On the Subscriber side, where the number of participating robots may be large or variable, the max samples per instance value should be set high enough to keep minimal data for each robot instance.

<hr class="hr-dashed">

## 12. LIFESPAN (LFSPAN)

> **How long a sample published by a Publisher remains valid**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>duration</code> (default: infinity)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Example
The LFSPAN QoS can be used to prevent robots from retaining outdated samples unnecessarily. For data such as po sition or battery level, where only the last few seconds matter, setting the duration accordingly ensures that the Publisher’s HistoryCache stores only the most recent samples, with older ones automatically deleted to conserve memory. In contrast, for data such as command logs– where long-term delivery is more important than freshness– it is preferable to set duration to infinity and rely on other QoS policies to ensure reliability.

<hr class="hr-dashed">

## 13. OWNERSHIP (+STRENGTH) (OWNST)

> **Whether multiple Publishers can concurrently update the same instance, or, if not, which Publisher’s value should be accepted**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>kind</code> (default: SHARED)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can not be changed at runtime</span>
  </div>
</div>

### kind Mode
* **SHARED**: Allows multiple Publishers to freely update the same instance.
* **EXCLUSIVE**: Only a single Publisher is allowed to update an instance, and its updates alone are delivered to Subscribers.<br>Priority is determined by the value of the OWNST STRENGTH QoS policy.

### Example
The OWNSTandOWNERSHIP STRENGTHQoSpolicies can be used to manage shared resources or mission states con sistently. For example, if multiple robots scan the environment simultaneously to build a shared map, shared mode allows the server to receive updates from all robots and generate a unified map. In contrast, for tasks that must be performed by a single robot, exclusive mode can be used with appropriate value assigned to each Publisher, ensuring that the active robot becomes the instance owner. If the current owner robot fails to respond due to a fault, DDS automatically transfers ownership to the standby robot with the next highest value, allowing the task to continue without interruption.

<hr class="hr-dashed">

## 14. DESTINATION ORDER (DESTORD)

> **How a Subscriber orders samples from multiple Publishers targeting the same instance**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>kind</code> (default: BY_RECEPTION_TIMESTAMP)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can not be changed at runtime</span>
  </div>
</div>

### Mode
* **BY_RECEPTION_TIMESTAMP**: Orders samples by their reception time at the Subscriber.
* **BY_SOURCE_TIMESTAMP**: Uses the timestamp assigned by the Publisher at publication, preserving the original creation order regardless of delays.

### Example
The DESTORD QoS policy can be used to maintain data consistency when multiple robots simultaneously update the same instance. By configuring by source timestamp, samples are ordered by their creation time regardless of network delays or arrival order, allowing all robots to share a consistent view of the map. In contrast, for real-time data such as current posi tions, where the latest received value is most important, using by reception timestamp enables the Subscriber to reflect the earliest arriving sample immediately.

<hr class="hr-dashed">

## 15. WRITER DATA LIFECYCLE (WDLIFE)

> **whether a Publisher should notify Subscribers with a dispose() when it unregisters an instance**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autodispose unregistered instances</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Mode
* **TRUE**: An instance to be automatically marked as disposed when an unregister() is called, so that it is recognized as deleted on the Subscriber side.
* **FALSE**: The unregister() only disassociates the writer from the instance without disposing of it; the application must explicitly call dispose() to fully delete the instance.

### Example
The WDLIFE QoS can be used to explicitly manage the lifecycle of object-based tasks. For example, when a robot detects an object with its sensors, it can publish the object’s key, position, type, and status on a topic, allowing other robots or the control system to subscribe and build a shared environment model. When the task is completed, the post processing behavior depends on the detecting robot’s autodis pose unregistered instances setting. If true, the instance is immediately disposed of and removed from the map or marked as “processed”; if false, only the Publisher-instance link is removed while the object information remains active, allowing another robot to rediscover and update the same object, thereby improving collaboration flexibility.


<hr class="hr-dashed">

## 16. READER DATA LIFECYCLE (RDLIFE)

> **How long a Subscriber retains samples that have been disposed of or are no longer associated with any Publisher**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter1</span>
    <span class="req-value"><code>autopurge disposed samples delay</code> (default: infinity)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Parameter2</span>
    <span class="req-value"><code>autopurge no writer samples delay</code> (default: infinity)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime</span>
  </div>
</div>

### Example
The RDLIFE QoS policy enables tiered instance man agement for improved efficiency. For example, in a tempo rary storage area where hundreds of pallets move quickly and historical positions become obsolete immediately, set ting autopurge disposed samples delay to 0 seconds ensures that the cache is cleared as soon as a robot calls dis pose(). This keeps the cache lightweight and prevents un necessary memory growth. In contrast, for static, critical objects awaiting inspection after relocation, setting autop urge no writer samples delay to 300 seconds allows newly joined robots to continue inspection even after brief com munication interruptions. By adjusting the reader-side delay values based on context, essential data can be preserved while irrelevant information is promptly discarded, ensuring efficient use of system resources.

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
