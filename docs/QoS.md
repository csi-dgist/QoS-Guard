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
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
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
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* Publisher and Subscriber are matched only if they share at least one common partition name in the irrespective lists.

### Example
The PART QoS can be used to separate identical data types into multiple logical groups without the need to define additional topics or create new domains. For instance, delivery robots and inventory robots may share common topics such as “status” and “command” within the same domain, but still require distinct data flows. By setting the names=delivery for the delivery robots and names=inventory for the inventory robots, a central management system can subscribe only to the desired partition and selectively receive data from a specific group of robots.

<hr class="hr-dashed">

## 3. USER DATA (USRDATA)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

<hr class="hr-dashed">

## 4. GROUP DATA (GRPDATA)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 


<hr class="hr-dashed">

## 5. TOPIC DATA (TOPDATA)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 


<hr class="hr-dashed">

## 6. RELIABILITY (RELIAB)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 


<hr class="hr-dashed">

## 7. DURABILITY (DURABL)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

<hr class="hr-dashed">

## 8. DEADLINE (DEADLN)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

<hr class="hr-dashed">

## 9. LIVELINESS (LIVENS)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

<hr class="hr-dashed">

## 10. HISTORY (HIST)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

<hr class="hr-dashed">

## 11. RESOURCE LIMITS (RESLIM)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

<hr class="hr-dashed">

## 12. LIFESPAN (LFSPAN)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 


<hr class="hr-dashed">

## 13. OWNERSHIP (+STRENGTH) (OWNST)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

<hr class="hr-dashed">

## 14. DESTINATION ORDER (DESTORD)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

<hr class="hr-dashed">

## 15. WRITER DATA LIFECYCLE (WDLIFE)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 


<hr class="hr-dashed">

## 16. READER DATA LIFECYCLE (RDLIFE)

> **Controls whether newly createdDDS entities automatically start participating in discovery**

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Parameter</span>
    <span class="req-value"><code>autoenable_created_entities</code> (default: TRUE)</span>
  </div>
  <div class="req-item">
    <span class="req-label">Mutability</span>
    <span class="req-value">Can be changed at runtime<br>changes affect only entities created after the update</span>
  </div>
</div>

### Mode
* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

### Example
The ENTFAC 

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
