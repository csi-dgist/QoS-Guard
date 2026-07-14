# DDS QoS Policy Guide

> A plain-language reference to the 16 DDS QoS policies that shape every ROS 2 topic.

QoS policies are the settings that decide how a topic behaves, for example whether delivery is guaranteed, whether a node that joins late still receives past messages, and how long each message stays in the queue. Every policy has its own page here that explains what it does, the values it accepts, and how it acts at each stage of a connection. Start anywhere or read straight through, then follow the links to the [Dependency Map](../chain.md) to see how policies affect one another, or to [Rules & Evidence](../rules/index.md) to see which combinations QoS Guard flags.

<style>
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
.req-item:last-child { border-bottom: none; }
.req-label {
    min-width: 100px;
    font-weight: 700;
    color: #334155;
    font-size: 13px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
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
b { color: #394a5b; }
</style>

<hr class="hr-grad-left">

## 16 QoS Policies

<div class="qos-toc-grid">
  <a href="entity-factory/" class="qos-card"><span class="qos-num">01</span><div class="qos-title">ENTITY FACTORY<span class="qos-abbr">ENTFAC — discovery timing</span></div></a>
  <a href="partition/" class="qos-card"><span class="qos-num">02</span><div class="qos-title">PARTITION<span class="qos-abbr">PART — logical segmentation</span></div></a>
  <a href="user-data/" class="qos-card"><span class="qos-num">03</span><div class="qos-title">USER DATA<span class="qos-abbr">USRDATA — participant metadata</span></div></a>
  <a href="group-data/" class="qos-card"><span class="qos-num">04</span><div class="qos-title">GROUP DATA<span class="qos-abbr">GRPDATA — pub/sub metadata</span></div></a>
  <a href="topic-data/" class="qos-card"><span class="qos-num">05</span><div class="qos-title">TOPIC DATA<span class="qos-abbr">TOPDATA — topic metadata</span></div></a>
  <a href="reliability/" class="qos-card"><span class="qos-num">06</span><div class="qos-title">RELIABILITY<span class="qos-abbr">RELIAB — delivery guarantee</span></div></a>
  <a href="durability/" class="qos-card"><span class="qos-num">07</span><div class="qos-title">DURABILITY<span class="qos-abbr">DURABL — late joiners</span></div></a>
  <a href="deadline/" class="qos-card"><span class="qos-num">08</span><div class="qos-title">DEADLINE<span class="qos-abbr">DEADLN — temporal constraints</span></div></a>
  <a href="liveliness/" class="qos-card"><span class="qos-num">09</span><div class="qos-title">LIVELINESS<span class="qos-abbr">LIVENS — publisher activity</span></div></a>
  <a href="history/" class="qos-card"><span class="qos-num">10</span><div class="qos-title">HISTORY<span class="qos-abbr">HIST — sample retention</span></div></a>
  <a href="resource-limits/" class="qos-card"><span class="qos-num">11</span><div class="qos-title">RESOURCE LIMITS<span class="qos-abbr">RESLIM — cache bounds</span></div></a>
  <a href="lifespan/" class="qos-card"><span class="qos-num">12</span><div class="qos-title">LIFESPAN<span class="qos-abbr">LFSPAN — sample validity</span></div></a>
  <a href="ownership/" class="qos-card"><span class="qos-num">13</span><div class="qos-title">OWNERSHIP<span class="qos-abbr">OWNST — multiple writers</span></div></a>
  <a href="destination-order/" class="qos-card"><span class="qos-num">14</span><div class="qos-title">DESTINATION ORDER<span class="qos-abbr">DESTORD — sample ordering</span></div></a>
  <a href="writer-data-lifecycle/" class="qos-card"><span class="qos-num">15</span><div class="qos-title">WRITER DATA LIFECYCLE<span class="qos-abbr">WDLIFE — instance cleanup</span></div></a>
  <a href="reader-data-lifecycle/" class="qos-card"><span class="qos-num">16</span><div class="qos-title">READER DATA LIFECYCLE<span class="qos-abbr">RDLIFE — purge timing</span></div></a>
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

<hr class="hr-grad-left">

## Next steps

| Section | Use it for |
|:---|:---|
| [Dependency Map](../chain.md) | See how the policies affect one another across all 40 rules |
| [Rules & Evidence](../rules/index.md) | Open each rule with its conflict condition, engine checks, and the evidence behind it |
