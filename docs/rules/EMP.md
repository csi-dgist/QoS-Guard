# EMP Rules

This page describes the QoS dependency rules derived from **Empirical analysis and experimental results**. These rules focus on runtime performance, network conditions (e.g., RTT), and timing-critical dependencies that were validated through systematic testing.

---

## Stage 1
*Intn.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| [6](#rule-6) | LFSPAN → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [LFSPAN.duration > 0]$ | Functional | Pub | EMP |

---

## Stage 3
*Ru.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| [31](#rule-31) | HIST → RELIAB | $[RELIABLE] \wedge [KEEP\_LAST] \wedge [depth < \lceil RTT/PP \rceil + 2]$ | Functional | Pub | EMP |
| [32](#rule-32) | RESLIM → RELIAB | $[RELIABLE] \wedge [KEEP\_ALL] \wedge [mpi < \lceil RTT/PP \rceil + 1]$ | Functional | Pub | EMP |
| [33](#rule-33) | LFSPAN → RELIAB | $[RELIABLE] \wedge [LFSPAN.duration < RTT \times 2]$ | Functional | Pub | EMP |
| [36](#rule-36) | LIVENS → DEADLN | $[DEADLN.period > 0] \wedge [LIVENS.lease < DEADLN.period]$ | Functional | Sub | EMP |
| [37](#rule-37) | HIST → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [KEEP\_ALL] \wedge [mpi \ge default]$ | Operational | Pub | EMP |
| [38](#rule-38) | DEADLN → OWNST | $[OWNST = EXCLUSIVE] \wedge [DEADLN.period < 2 \times PP]$ | Operational | Sub | EMP |
| [39](#rule-39) | LIVENS → OWNST | $[OWNST = EXCLUSIVE] \wedge [lease < 2 \times PP]$ | Operational | Sub | EMP |
| [40](#rule-40) | DURABL → DEADLN | $[DEADLN.period > 0] \wedge [DURABL \ge TRAN\_LOCAL]$ | Operational | Sub | EMP |

---

*(Note: mpi = max_samples_per_instance, PP = Publish Period, RTT = Round Trip Time)*

##  Experimental Evidence Details
---

### Rule 6
*Validates why Durability (Transient Local) requires a non-zero Lifespan to provide late-joining data.*

**1. Experimental Setup**

* **Publisher:** Durability = `TRANSIENT_LOCAL`, Lifespan = `50ms`
* **Subscriber 1 (Existing):** Launched before Publisher.
* **Subscriber 2 (Late-joiner):** Launched after Publisher finishes sending 1,000 samples.
* **Total Samples Sent:** 1,000

**2. Test Scenario (Step-by-Step)**

1.  Launch **Subscriber 1** to monitor live data.
2.  Launch **Publisher** and transmit 1,000 samples (Total time taken > 50ms).
3.  Confirm **Subscriber 1** received all 1,000 samples.
4.  Launch **Subscriber 2** (Late-joiner) to retrieve historical data from the Publisher's buffer.

**3. Experimental Observation**

| Entity | Expected Received | Actual Received | Status |
| :--- | :---: | :---: | :---: |
| Subscriber 1 (Live) | 1,000 | 1,000 | ✅ Success |
| Subscriber 2 (Late) | 1,000 | **0** | ❌ Data Expired |

**4. Empirical Conclusion**

Even though `TRANSIENT_LOCAL` is set to store data for late-joiners, the **Lifespan (50ms)** caused all buffered samples to be purged from the Publisher's queue before Subscriber 2 could connect.

---

### Rule 31 
*Justifies the minimum History Depth required to prevent data loss in Reliable communication under network delay and loss.*

**1. Experimental Setup**

* **QoS Profile:** Reliability = `RELIABLE`, History Kind = `KEEP_LAST`
* **Network Condition (Loss):** 5% Packet Loss (Simulated via `tc`)
* **Network Condition (Delay):** 100ms to 500ms RTT (Round Trip Time)
* **Publication Period (PP):** 100ms (10Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Test Scenario (Step-by-Step)**

1.  Set the network packet loss to 5% and RTT to a range of 100ms to 500ms.
2.  Transmit 1,000 samples from Publisher to Subscriber.
3.  Decrease the History Depth incrementally for each test run.

**3. Experimental Observation**

<p align="center">
  <img src="../images/rule31.png" style="width: 500px; height: auto;">
  <br>
  <em>Figure: Message loss rate analysis according to History Depth and RTT</em>
</p>

**4. Empirical Conclusion**

In a lossy network (5% loss), a Reliable connection requires retransmission of lost packets. If the **History Depth** is smaller than the number of samples sent during one **RTT**, the buffer is overwritten before a retransmission can be requested. 

---
### Rule 32
*Justifies the minimum Resource Limits (max_samples_per_instance) required to sustain reliable transmission under network delay.*

**1. Experimental Setup**

* **QoS Profile:** Reliability = `RELIABLE`, History Kind = `KEEP_ALL`
* **Resource Limits:** `max_samples_per_instance` (Variable: $1 \sim N$)
* **Network Condition:** packet loss 5%
* **Network Latency (RTT):** 100ms, 200ms, 300ms, 400ms (Simulated via `tc`)
* **Publication Period (PP):** 100ms (10Hz)

**2. Test Scenario (Step-by-Step)**

1.  Connect two notebooks and verify the baseline RTT.
2.  Set the Publisher's History to `KEEP_ALL` to ensure all samples are subject to Resource Limits.
3.  Vary the RTT from 100ms to 400ms using network emulation tools.
4.  Decrease `max_samples_per_instance` until sample rejected or lost events occur.
5.  Record the minimum `mpi` value that ensures 100% successful delivery.

**3. Experimental Observation**

![Rule 32](../images/rule32.png){ width="100" }

**4. Empirical Conclusion**

When using `KEEP_ALL`, the `max_samples_per_instance` (mpi) acts as the effective buffer size for the reliability protocol. If mpi is insufficient to hold all samples sent during one **RTT** (plus the time for ACK/NACK processing), the Publisher will either block or drop samples.


---
### Rule 33
*Validates why Durability (Transient Local) requires a non-zero Lifespan to provide late-joining data.*

**1. Experimental Setup**

* **Publisher:** Durability = `TRANSIENT_LOCAL`, Lifespan = `50ms`
* **Subscriber 1 (Existing):** Launched before Publisher.
* **Subscriber 2 (Late-joiner):** Launched after Publisher finishes sending 1,000 samples.
* **Total Samples Sent:** 1,000

**2. Test Scenario (Step-by-Step)**

1.  Launch **Subscriber 1** to monitor live data.
2.  Launch **Publisher** and transmit 1,000 samples (Total time taken > 50ms).
3.  Confirm **Subscriber 1** received all 1,000 samples.
4.  Launch **Subscriber 2** (Late-joiner) to retrieve historical data from the Publisher's buffer.

**3. Experimental Observation**

| Entity | Expected Received | Actual Received | Status |
| :--- | :---: | :---: | :---: |
| Subscriber 1 (Live) | 1,000 | 1,000 | ✅ Success |
| Subscriber 2 (Late) | 1,000 | **0** | ❌ Data Expired |

**4. Empirical Conclusion**

Even though `TRANSIENT_LOCAL` is set to store data for late-joiners, the **Lifespan (50ms)** caused all buffered samples to be purged from the Publisher's queue before Subscriber 2 could connect.

---
### Rule 36 
*Validates why Durability (Transient Local) requires a non-zero Lifespan to provide late-joining data.*

**1. Experimental Setup**

* **Publisher:** Durability = `TRANSIENT_LOCAL`, Lifespan = `50ms`
* **Subscriber 1 (Existing):** Launched before Publisher.
* **Subscriber 2 (Late-joiner):** Launched after Publisher finishes sending 1,000 samples.
* **Total Samples Sent:** 1,000

**2. Test Scenario (Step-by-Step)**

1.  Launch **Subscriber 1** to monitor live data.
2.  Launch **Publisher** and transmit 1,000 samples (Total time taken > 50ms).
3.  Confirm **Subscriber 1** received all 1,000 samples.
4.  Launch **Subscriber 2** (Late-joiner) to retrieve historical data from the Publisher's buffer.

**3. Experimental Observation**

| Entity | Expected Received | Actual Received | Status |
| :--- | :---: | :---: | :---: |
| Subscriber 1 (Live) | 1,000 | 1,000 | ✅ Success |
| Subscriber 2 (Late) | 1,000 | **0** | ❌ Data Expired |

**4. Empirical Conclusion**

Even though `TRANSIENT_LOCAL` is set to store data for late-joiners, the **Lifespan (50ms)** caused all buffered samples to be purged from the Publisher's queue before Subscriber 2 could connect.

---
### Rule 37
*Validates why Durability (Transient Local) requires a non-zero Lifespan to provide late-joining data.*

**1. Experimental Setup**

* **Publisher:** Durability = `TRANSIENT_LOCAL`, Lifespan = `50ms`
* **Subscriber 1 (Existing):** Launched before Publisher.
* **Subscriber 2 (Late-joiner):** Launched after Publisher finishes sending 1,000 samples.
* **Total Samples Sent:** 1,000

**2. Test Scenario (Step-by-Step)**

1.  Launch **Subscriber 1** to monitor live data.
2.  Launch **Publisher** and transmit 1,000 samples (Total time taken > 50ms).
3.  Confirm **Subscriber 1** received all 1,000 samples.
4.  Launch **Subscriber 2** (Late-joiner) to retrieve historical data from the Publisher's buffer.

**3. Experimental Observation**

| Entity | Expected Received | Actual Received | Status |
| :--- | :---: | :---: | :---: |
| Subscriber 1 (Live) | 1,000 | 1,000 | ✅ Success |
| Subscriber 2 (Late) | 1,000 | **0** | ❌ Data Expired |

**4. Empirical Conclusion**

Even though `TRANSIENT_LOCAL` is set to store data for late-joiners, the **Lifespan (50ms)** caused all buffered samples to be purged from the Publisher's queue before Subscriber 2 could connect.

---
### Rule 38
*Validates why Durability (Transient Local) requires a non-zero Lifespan to provide late-joining data.*

**1. Experimental Setup**

* **Publisher:** Durability = `TRANSIENT_LOCAL`, Lifespan = `50ms`
* **Subscriber 1 (Existing):** Launched before Publisher.
* **Subscriber 2 (Late-joiner):** Launched after Publisher finishes sending 1,000 samples.
* **Total Samples Sent:** 1,000

**2. Test Scenario (Step-by-Step)**

1.  Launch **Subscriber 1** to monitor live data.
2.  Launch **Publisher** and transmit 1,000 samples (Total time taken > 50ms).
3.  Confirm **Subscriber 1** received all 1,000 samples.
4.  Launch **Subscriber 2** (Late-joiner) to retrieve historical data from the Publisher's buffer.

**3. Experimental Observation**

| Entity | Expected Received | Actual Received | Status |
| :--- | :---: | :---: | :---: |
| Subscriber 1 (Live) | 1,000 | 1,000 | ✅ Success |
| Subscriber 2 (Late) | 1,000 | **0** | ❌ Data Expired |

**4. Empirical Conclusion**

Even though `TRANSIENT_LOCAL` is set to store data for late-joiners, the **Lifespan (50ms)** caused all buffered samples to be purged from the Publisher's queue before Subscriber 2 could connect.

---
### Rule 39
*Validates why Durability (Transient Local) requires a non-zero Lifespan to provide late-joining data.*

**1. Experimental Setup**

* **Publisher:** Durability = `TRANSIENT_LOCAL`, Lifespan = `50ms`
* **Subscriber 1 (Existing):** Launched before Publisher.
* **Subscriber 2 (Late-joiner):** Launched after Publisher finishes sending 1,000 samples.
* **Total Samples Sent:** 1,000

**2. Test Scenario (Step-by-Step)**

1.  Launch **Subscriber 1** to monitor live data.
2.  Launch **Publisher** and transmit 1,000 samples (Total time taken > 50ms).
3.  Confirm **Subscriber 1** received all 1,000 samples.
4.  Launch **Subscriber 2** (Late-joiner) to retrieve historical data from the Publisher's buffer.

**3. Experimental Observation**

| Entity | Expected Received | Actual Received | Status |
| :--- | :---: | :---: | :---: |
| Subscriber 1 (Live) | 1,000 | 1,000 | ✅ Success |
| Subscriber 2 (Late) | 1,000 | **0** | ❌ Data Expired |

**4. Empirical Conclusion**

Even though `TRANSIENT_LOCAL` is set to store data for late-joiners, the **Lifespan (50ms)** caused all buffered samples to be purged from the Publisher's queue before Subscriber 2 could connect.

---
### Rule 40
*Validates why Durability (Transient Local) requires a non-zero Lifespan to provide late-joining data.*

**1. Experimental Setup**

* **Publisher:** Durability = `TRANSIENT_LOCAL`, Lifespan = `50ms`
* **Subscriber 1 (Existing):** Launched before Publisher.
* **Subscriber 2 (Late-joiner):** Launched after Publisher finishes sending 1,000 samples.
* **Total Samples Sent:** 1,000

**2. Test Scenario (Step-by-Step)**

1.  Launch **Subscriber 1** to monitor live data.
2.  Launch **Publisher** and transmit 1,000 samples (Total time taken > 50ms).
3.  Confirm **Subscriber 1** received all 1,000 samples.
4.  Launch **Subscriber 2** (Late-joiner) to retrieve historical data from the Publisher's buffer.

**3. Experimental Observation**

| Entity | Expected Received | Actual Received | Status |
| :--- | :---: | :---: | :---: |
| Subscriber 1 (Live) | 1,000 | 1,000 | ✅ Success |
| Subscriber 2 (Late) | 1,000 | **0** | ❌ Data Expired |

**4. Empirical Conclusion**

Even though `TRANSIENT_LOCAL` is set to store data for late-joiners, the **Lifespan (50ms)** caused all buffered samples to be purged from the Publisher's queue before Subscriber 2 could connect.

---


## 📝 Notation Summary
* **mpi**: `max_samples_per_instance`
* **PP**: `Publish Period` (Time interval between consecutive samples)
* **RTT**: `Round Trip Time` (Network latency)
