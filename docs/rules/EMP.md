# EMP Rules

<style>
.md-typeset table td:first-child {
  text-align: center;
  padding: 8px !important;  
}

.md-typeset table td:first-child a {
  display: inline-block; 
  width: 24px;         
  height: 24px;        
  line-height: 24px;    
  
  border-radius: 50%;   
  background-color: #f0f0f0; 
  color: #000 !important;
  text-decoration: none !important;
  font-weight: bold;
  font-size: 13px;      
  transition: 0.2s;
}

.md-typeset table tr:hover td:first-child a {
  background-color: #4e37e6 !important; 
  color: #fff !important; 
  transform: scale(1.1);
}

/* std-list card style (same as IMP) */
.std-list { display: flex; flex-direction: column; gap: 16px; margin: 24px 0; }
.std-item { border: 1px solid #e1e4e8; border-radius: 10px; background: #ffffff; padding: 16px 20px; transition: all 0.3s cubic-bezier(0.25, 0.8, 0.25, 1); position: relative; text-decoration: none !important; color: inherit !important; }
.std-item:hover { box-shadow: 0 8px 16px rgba(0,0,0,0.1); border-color: #4e37e6; transform: translateY(-2px); }
.std-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
.std-no { background: #f0f0f0; color: #444; width: 28px; height: 28px; display: flex; align-items: center; justify-content: center; border-radius: 50%; font-weight: 800; font-size: 13px; transition: 0.3s; }
.std-item:hover .std-no { background: #4e37e6; color: #fff; }
.std-id { font-weight: 700; font-size: 1.05em; color: #2c3e50; flex-grow: 1; margin-left: 12px; }
.std-condition { background: #f8f9fa; padding: 14px; border-radius: 6px; font-family: 'Consolas', 'Monaco', monospace; border-left: 4px solid #4e37e6; margin: 8px 0; font-size: 0.95em; color: #000000; line-height: 1.5; }
.std-footer { display: flex; flex-wrap: wrap; gap: 12px; margin-top: 10px; font-size: 0.82em; }
.std-tag { display: flex; align-items: center; background: #f1f3f5; padding: 2px 8px; border-radius: 4px; color: #666; }
.std-tag b { color: #4e37e6; margin-right: 5px; font-weight: 600; }
</style>


This page describes the QoS dependency rules derived from **Empirical analysis and experimental results**. These rules focus on runtime performance, network conditions (e.g., RTT), and timing-critical dependencies that were validated through systematic testing.

<hr class="hr-grad-left">

## Stage 1
*Intra-entity Dependency Validation*

<div class="std-list">

  <a href="#rule-6" class="std-item">
    <div class="std-header">
      <span class="std-no">6</span>
      <span class="std-id">LFSPAN → DURABL</span>
      <span style="font-size: 0.8em; color: #999;">Functional</span>
    </div>
    <div class="std-condition">
      [DURABL ≥ TRAN_LOCAL] ∧ [LFSPAN.duration > 0]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Pub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

</div>

<hr class="hr-grad-left">

## Stage 3
*Timing-based Dependency Validation*

<div class="std-list">

  <a href="#rule-31" class="std-item">
    <div class="std-header">
      <span class="std-no">31</span>
      <span class="std-id">HIST → RELIAB</span>
      <span style="font-size: 0.8em; color: #999;">Functional</span>
    </div>
    <div class="std-condition">
      [RELIABLE] ∧ [HIST.kind = KEEP_LAST] ∧ [HIST.depth < ⌈ 2 × RTT/PP ⌉ + 1]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Pub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

  <a href="#rule-32" class="std-item">
    <div class="std-header">
      <span class="std-no">32</span>
      <span class="std-id">RESLIM → RELIAB</span>
      <span style="font-size: 0.8em; color: #999;">Functional</span>
    </div>
    <div class="std-condition">
      [RELIABLE] ∧ [HIST.kind = KEEP_ALL] ∧ [RESLIM.mpi < ⌈ 2 × RTT/PP ⌉ + 1]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Pub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

  <a href="#rule-33" class="std-item">
    <div class="std-header">
      <span class="std-no">33</span>
      <span class="std-id">LFSPAN → RELIAB</span>
      <span style="font-size: 0.8em; color: #999;">Functional</span>
    </div>
    <div class="std-condition">
      [RELIABLE] ∧ [LFSPAN.duration < PP + 2 × RTT]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Pub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

  <a href="#rule-35" class="std-item">
    <div class="std-header">
      <span class="std-no">35</span>
      <span class="std-id">RELIAB → DEADLN</span>
      <span style="font-size: 0.8em; color: #999;">Functional</span>
    </div>
    <div class="std-condition">
      [DEADLN.period > 0] ∧ [RELIAB = BEST_EFFORT]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Sub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

  <a href="#rule-36" class="std-item">
    <div class="std-header">
      <span class="std-no">36</span>
      <span class="std-id">LIVENS → DEADLN</span>
      <span style="font-size: 0.8em; color: #999;">Functional</span>
    </div>
    <div class="std-condition">
      [DEADLN.period > 0] ∧ [LIVENS.lease < DEADLN.period]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Sub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

  <a href="#rule-37" class="std-item">
    <div class="std-header">
      <span class="std-no">37</span>
      <span class="std-id">HIST → DURABL</span>
      <span style="font-size: 0.8em; color: #999;">Operational</span>
    </div>
    <div class="std-condition">
      [DURABL ≥ TRAN_LOCAL] ∧ [HIST.kind = KEEP_ALL] ∧ [RESLIM.mpi ≥ default]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Pub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

  <a href="#rule-38" class="std-item">
    <div class="std-header">
      <span class="std-no">38</span>
      <span class="std-id">DEADLN → OWNST</span>
      <span style="font-size: 0.8em; color: #999;">Operational</span>
    </div>
    <div class="std-condition">
      [OWNST = EXCLUSIVE] ∧ [DEADLN.period < PP + 2 × RTT]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Sub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

  <a href="#rule-39" class="std-item">
    <div class="std-header">
      <span class="std-no">39</span>
      <span class="std-id">LIVENS → OWNST</span>
      <span style="font-size: 0.8em; color: #999;">Operational</span>
    </div>
    <div class="std-condition">
      [OWNST = EXCLUSIVE] ∧ [LIVENS.lease < PP + 2 × RTT]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Sub</div>
      <div class="std-tag"><b>Basis</b> EMP</div>
    </div>
  </a>

</div>

<hr class="hr-grad-left">

!!! info "Notation Summary"
    * **mpi**: `max_samples_per_instance` 
    * **PP**: `Publish Period` 
    * **RTT**: `Round Trip Time`

<hr class="hr-double">

##  Experimental Evidence Details

<span id="rule-6"></span>
### Rule 6
*Let us prove that for late-joining data to be transmitted, the sample must not have expired.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** RELIABILITY(`RELIABLE`), HISTORY(`KEEP_ALL`)
* **Publisher:** Durability = `TRANSIENT_LOCAL`, Lifespan = `50ms`
* **Subscriber 1 (Existing):** Launched before Publisher.
* **Subscriber 2 (Late-joiner):** Launched after Publisher finishes sending 1,000 samples.
* **Total Samples Sent:** 1,000


**2. Test Scenario**

1.  Launch **Subscriber 1** to monitor live data.
2.  Launch **Publisher** and transmit 1,000 samples.
3.  Confirm **Subscriber 1** received all 1,000 samples.
4.  Launch **Subscriber 2** (Late-joiner) to retrieve historical data from the Publisher's buffer.

**3. Experimental Observation**

| Entity | Expected Received | Actual Received | Status |
| :--- | :---: | :---: | :---: |
| <font size="4">Subscriber 1</font> | <font size="4">1,000</font> | <font size="4">1,000</font> | <font size="4">✅ Success</font> |
| <font size="4">Subscriber 2 (Late)</font> | <font size="4">1,000</font> | <font size="4">**0**</font> | <font size="4">❌ Data Expired</font> |

**4. Empirical Conclusion**

Even though `TRANSIENT_LOCAL` is set to store data for late-joiners, the **Lifespan (`50ms`)** caused all buffered samples to be purged from the Publisher's queue before Subscriber 2 could connect.

<hr class="hr-dashed">

<span id="rule-31"></span>
### Rule 31 
*Justifies the minimum History Depth required to prevent data loss in Reliable communication under network delay and loss.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** RELIABILITY(`RELIABLE`), HISTORY(`KEEP_LAST`)
* **Network Condition (Loss):** 5% Packet Loss (Simulated via `tc`)
* **Network Condition (Delay):** 100ms to 500ms RTT (Round Trip Time)
* **Variable:** History Depth ($1 \sim N$)

**2. Test Scenario**

1.  Set the network packet loss to 5% and RTT to a range of 100ms to 500ms.
2.  Transmit 1,000 samples from Publisher to Subscriber.
3.  Decrease the History Depth incrementally for each test run. (10 to 1)

**3. Experimental Observation**

<div align="center">
  <img src="../../images/rule31.png" width="500">
</div>


**4. Empirical Conclusion**

In a lossy network (5% loss), a Reliable connection requires retransmission of lost packets. If the **History Depth** is smaller than the number of samples sent during one **RTT**, the buffer is overwritten before a retransmission can be requested. 

<hr class="hr-dashed">

<span id="rule-32"></span>
### Rule 32
*Justifies the minimum Resource Limits (max_samples_per_instance) required to sustain reliable transmission under network delay.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** RELIABILITY(`RELIABLE`), HISTORY(`KEEP_ALL`)
* **Resource Limits:** `max_samples_per_instance` (Variable: $1 \sim N$)
* **Network Condition (Loss):** 5% Packet Loss (Simulated via `tc`)
* **Network Condition (Delay):** 100ms to 500ms RTT (Round Trip Time)

**2. Test Scenario**

1.  Set the network packet loss to 5% and RTT to a range of 100ms to 500ms.
2.  Set the Publisher's History to `KEEP_ALL` to ensure all samples are subject to Resource Limits.
3.  Decrease `max_samples_per_instance` until sample rejected or lost events occur. (10 to 1)
4.  Record the minimum `mpi` value that ensures 100% successful delivery.

**3. Experimental Observation**

<div align="center">
  <img src="../../images/Rule32.png" width="500">
</div>

**4. Empirical Conclusion**

When using `KEEP_ALL`, the `max_samples_per_instance` (mpi) acts as the effective buffer size for the reliability protocol. If mpi is insufficient to hold all samples sent during one **RTT** (plus the time for ACK/NACK processing), the Publisher will block samples.

<hr class="hr-dashed">

<span id="rule-33"></span>
### Rule 33
*Justifies the minimum Lifespan duration required to ensure sample reception before expiration in reliable communication.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** RELIABILITY(`RELIABLE`), HISTORY(`KEEP_ALL`)
* **Publication Period (PP):** 10ms to 100ms
* **Lifespan Duration:** 100ms to 1000ms
* **Network Condition (Loss):** 20% Packet Loss (Simulated via `tc`)
* **Total Samples:** 10,000

**2. Test Scenario**

1.  Set the network packet loss to 20%.
2.  Perform a grid search by varying the **Publish Period (PP)** from 10ms to 100ms and **Lifespan duration** from 100ms to 1000ms.
3.  Measure the total number of samples successfully received at the Subscriber.
4.  Identify the threshold where sample reception starts to drop despite using `RELIABLE` QoS.

**3. Experimental Observation**

<div align="center">
  <img src="../../images/rule33.png" width="500">
</div>

* **High Reception Zone (Blue):** When Lifespan is sufficiently longer than PP + 2 x RTT, nearly all 10,000 samples are received.
* **Low Reception Zone (White/Gray):** When Lifespan duration is close to or shorter than PP + 2 x RTT, samples expire before they can be successfully delivered or processed.

**4. Empirical Conclusion**

Even with `RELIABLE` settings, data loss occurs if the **Lifespan duration** is shorter than the time required for successful transmission and acknowledgement. 

<hr class="hr-dashed">

<span id="rule-35"></span>
### Rule 35
*Justifies how Reliability settings impact Deadline compliance under packet loss conditions.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** HISTORY(`KEEP_ALL`)
* **Variables:** Deadline Period = `150ms`, Reliability = `RELIABLE` vs `BEST_EFFORT`
* **Network Condition (Loss):** 5% Packet Loss (Simulated via `tc`)
* **Total Samples:** 1,000

**2. Test Scenario**

1.  Set the Publisher and Subscriber with a Deadline period of 150ms.
2.  Introduce 5% packet loss to the network interface.
3.  Case A: Configure both entities with `RELIABLE` Reliability.
4.  Case B: Configure both entities with `BEST_EFFORT` Reliability.
5.  Publish 1,000 samples and monitor the `on_requested_deadline_missed` callback at the Subscriber.

**3. Experimental Observation**

<div align="center">
  <img src="../../images/Rule35.png" width="500">
</div>

| Reliability | Total Samples | Received Samples | Deadline Missed Count | Cause of Violation |
| :--- | :---: | :---: | :---: | :--- |
| **RELIABLE** | 1,000 | 1,000 | 25 | Delay caused by Retransmission |
| **BEST_EFFORT** | 1,000 | ~950 | 48 | Gap caused by Packet Loss |

**4. Empirical Conclusion**

The experimental results highlight two different causes of Deadline violations:<br>
1. In **RELIABLE** mode, all 1,000 samples were received, but 25 samples violated the Deadline because the time taken for NACK-based retransmission exceeded the 150ms window.<br>
2. In **BEST_EFFORT** mode, violations nearly doubled (48 times) because lost packets created permanent gaps in the data stream, directly triggering the Deadline timer.

<hr class="hr-dashed">

<span id="rule-36"></span>
### Rule 36
*Justifies the consistency requirement between Liveliness Lease Duration and Deadline Period to avoid redundant or conflicting fault detections.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** RELIABILITY(`RELIABLE`), HISTORY(`KEEP_ALL`)
* **Deadline Period:** 200ms
* **Total Samples:** 600 (Total duration: 60s)
* **Liveliness Lease Duration**: Variable
* * **Network Condition (Loss):** 100% (Simulated via `tc`)

**2. Test Scenario**

The network condition is controlled over 5 seconds using the `tc` command:<br>
**40s ~ 45s (Loss 100%):** Induce **Liveliness Lost** by blocking all packets.


**3. Experimental Observation & Empirical Conclusion**

**[RESULT]**
The experiment demonstrates that when `Lease Duration < Deadline`, the system falls into a contradictory state: it continues to fire "Data Missing" alarms (Deadline Missed) for a Publisher that it has already declared "Dead" (Liveliness Lost). 

To prevent this **state inconsistency** and ensure a logical fault-detection sequence (where the data stream is monitored within the lifespan of the entity), the Liveliness Lease Duration must always be longer than the Deadline Period:
$LIVENS.lease ≥ DEADLN.period$

<hr class="hr-dashed">

<span id="rule-37"></span>
### Rule 37
*Justifies the impact of Resource Limits on the recovery latency for late-joining subscribers in durable communication.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** RELIABILITY(`RELIABLE`), HISTORY(`KEEP_ALL`), DURABILITY(`TRANSIENT_LOCAL`)
* **Variable:** `max_samples_per_instance` (mpi) ranging from 1,000 to 5,000
* **Metric:** Latency (ms) - required for the system to reach steady-state convergence

**2. Test Scenario**

1.  Initialize a Publisher that sends samples continuously with `TRANSIENT_LOCAL` durability.
2.  After a specific number of samples are published, introduce a Late-Joining Subscriber.
3.  Vary the **max_samples_per_instance** setting on the Pubscriber side.
4.  Measure the latency between the sample generation time and the time it is actually processed by the late-joiner.

**3. Experimental Observation**

<div align="center">
  <img src="../../images/rule37.png" width="500">
</div>

* **Low mpi Zone:** Recovery is relatively fast
* **High mpi Zone:** As `max_samples_per_instance` increases towards 5,000, the recovery latency increases exponentially (reaching up to 50,000ms). The system takes significantly longer to converge to a steady state.

**4. Empirical Conclusion**

The experiment proves that setting an excessively high `max_samples_per_instance` for the sake of data completeness leads to a **"Recovery Latency Storm."** For a late-joining node, processing thousands of buffered historical samples simultaneously saturates the available bandwidth and CPU, causing current data to be delayed by tens of seconds.

<hr class="hr-dashed">

<span id="rule-38"></span>
### Rule 38
*Demonstrates that setting the DEADLN.period too short in EXCLUSIVE mode can result in unintended ownership transfers.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** RELIABILITY(`RELIABLE`), HISTORY(`KEEP_ALL`), OWNERSHIP(`EXCLUSIVE`)
* **Variable:** Deadline Period (`100ms` vs `500ms`)
* **Network Condition (Loss):** 5% Packet Loss (Simulated via `tc`)


**2. Test Scenario**

1.  Configure two Publishers with different strengths for `EXCLUSIVE` ownership.
2.  Introduce a constant 5% packet loss using the `tc` command.
3.  **Case A:** Set the Deadline period to `100ms`.
4.  **Case B:** Set the Deadline period to `500ms`.
5.  Monitor the `on_requested_deadline_missed` callback and track how often the active owner is switched due to deadline timeouts.

**3. Experimental Observation**

<div align="center">
  <img src="../../images/rule38.png" width="500">
</div>

* **Case A (Deadline = 100ms - red):** Numerous spikes are observed throughout the experiment. Any minor retransmission delay caused by the 5% loss immediately triggers a deadline miss, leading to unstable ownership.
* **Case B (Deadline = 500ms - green):** Significantly fewer and more sparse spikes appear. The larger window provides sufficient time for the `RELIABLE` protocol to recover lost packets through NACK/retransmission before the 500ms timer expires, maintaining a more consistent owner state.

**4. Empirical Conclusion**

The experiment clearly demonstrates that **Ownership Stability** is highly dependent on the **Deadline** period in lossy networks. A tight deadline (Case A) causes "Ownership Churning," where control is frequently and unnecessarily handed over due to transient network jitters. 

<hr class="hr-dashed">

<span id="rule-39"></span>
### Rule 39
*Demonstrates that, in EXCLUSIVE mode, setting the lease duration for Liveliness too short can result in unintended transfers of ownership.*

**1. Experimental Setup**

* **DATA:** 1kB, 10Hz
* **QoS:** RELIABILITY(`RELIABLE`), HISTORY(`KEEP_ALL`), OWNERSHIP(`EXCLUSIVE`), LIVELINESS(`AUTOMATIC`)
* **Variable (Lease Duration):** 50ms, 100ms, 300ms, 500ms, 800ms
* **Network Condition (Loss):** 100% (Simulated via `tc`)
* **Metric:** Total count of Liveliness Lost events

**2. Test Scenario**

1.  Initialize two Publishers with different strengths for `EXCLUSIVE` ownership.
2.  Apply a harsh network environment with **100% packet loss** using the `tc` command.
3.  Publish a total of **10,000 samples** to observe long-term stability.
4.  Vary the **Liveliness Lease Duration** from 50ms to 800ms.
5.  Record the frequency of ownership handovers triggered by `on_liveliness_changed` at the Subscriber side.

**3. Experimental Observation**

<div align="center">
  <img src="../../images/rule39.png" width="500">
</div>

* **Short Lease Duration (50ms - 100ms):** A high frequency of Liveliness Lost events is observed (up to 26 times). Since the lease duration is shorter than or equal to the Publication Period (100ms), even a single delayed heartbeat triggers a failure detection.
* **Long Lease Duration (300ms - 800ms):** The event count drops significantly as the lease duration increases. At 800ms, only 10 events occur, indicating that the system can tolerate multiple lost or delayed heartbeats without dropping the ownership.

**4. Empirical Conclusion**

The experiment highlights the risk of **"False Positive Failures."** When the Liveliness Lease Duration is too aggressive, transient network issues cause the Subscriber to frequently revoke ownership from the primary Publisher. This leads to unstable system control and unnecessary handovers.


<hr class="hr-double">
