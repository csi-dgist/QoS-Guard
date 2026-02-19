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
*History Depth vs. Network Latency*

**1. Experimental Setup**
* **Network Condition:** (e.g., Simulated RTT using `tc`)
* **Publication Rate (PP):** (e.g., 20ms / 50Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Scenario**
(Describe how the test was performed, e.g., "Measuring message loss rate while increasing RTT and decreasing History Depth.")

**3. Results & Graph**
![Rule 31 Graph](images/rule31_result.png)
> **Observation:** Data loss occurs when the buffer size is smaller than the outstanding unacknowledged samples required for retransmission.

**4. Empirical Formula Derivation**
* Based on the results, the minimum depth must satisfy: $depth \ge \lceil RTT/PP \rceil + 2$.

---
### Rule 32
*.*

**1. Experimental Setup**
* **Network Condition:** (e.g., Simulated RTT using `tc`)
* **Publication Rate (PP):** (e.g., 20ms / 50Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Scenario**
(Describe how the test was performed, e.g., "Measuring message loss rate while increasing RTT and decreasing History Depth.")

**3. Results & Graph**
![Rule 31 Graph](images/rule31_result.png)
> **Observation:** Data loss occurs when the buffer size is smaller than the outstanding unacknowledged samples required for retransmission.

**4. Empirical Formula Derivation**
* Based on the results, the minimum depth must satisfy: $depth \ge \lceil RTT/PP \rceil + 2$.

---
### Rule 33
*.*

**1. Experimental Setup**
* **Network Condition:** (e.g., Simulated RTT using `tc`)
* **Publication Rate (PP):** (e.g., 20ms / 50Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Scenario**
(Describe how the test was performed, e.g., "Measuring message loss rate while increasing RTT and decreasing History Depth.")

**3. Results & Graph**
![Rule 31 Graph](images/rule31_result.png)
> **Observation:** Data loss occurs when the buffer size is smaller than the outstanding unacknowledged samples required for retransmission.

**4. Empirical Formula Derivation**
* Based on the results, the minimum depth must satisfy: $depth \ge \lceil RTT/PP \rceil + 2$.

---
### Rule 36 
*.*

**1. Experimental Setup**
* **Network Condition:** (e.g., Simulated RTT using `tc`)
* **Publication Rate (PP):** (e.g., 20ms / 50Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Scenario**
(Describe how the test was performed, e.g., "Measuring message loss rate while increasing RTT and decreasing History Depth.")

**3. Results & Graph**
![Rule 31 Graph](images/rule31_result.png)
> **Observation:** Data loss occurs when the buffer size is smaller than the outstanding unacknowledged samples required for retransmission.

**4. Empirical Formula Derivation**
* Based on the results, the minimum depth must satisfy: $depth \ge \lceil RTT/PP \rceil + 2$.

---
### Rule 37
*.*

**1. Experimental Setup**
* **Network Condition:** (e.g., Simulated RTT using `tc`)
* **Publication Rate (PP):** (e.g., 20ms / 50Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Scenario**
(Describe how the test was performed, e.g., "Measuring message loss rate while increasing RTT and decreasing History Depth.")

**3. Results & Graph**
![Rule 31 Graph](images/rule31_result.png)
> **Observation:** Data loss occurs when the buffer size is smaller than the outstanding unacknowledged samples required for retransmission.

**4. Empirical Formula Derivation**
* Based on the results, the minimum depth must satisfy: $depth \ge \lceil RTT/PP \rceil + 2$.

---
### Rule 38
*.*

**1. Experimental Setup**
* **Network Condition:** (e.g., Simulated RTT using `tc`)
* **Publication Rate (PP):** (e.g., 20ms / 50Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Scenario**
(Describe how the test was performed, e.g., "Measuring message loss rate while increasing RTT and decreasing History Depth.")

**3. Results & Graph**
![Rule 31 Graph](images/rule31_result.png)
> **Observation:** Data loss occurs when the buffer size is smaller than the outstanding unacknowledged samples required for retransmission.

**4. Empirical Formula Derivation**
* Based on the results, the minimum depth must satisfy: $depth \ge \lceil RTT/PP \rceil + 2$.

---
### Rule 39
*.*

**1. Experimental Setup**
* **Network Condition:** (e.g., Simulated RTT using `tc`)
* **Publication Rate (PP):** (e.g., 20ms / 50Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Scenario**
(Describe how the test was performed, e.g., "Measuring message loss rate while increasing RTT and decreasing History Depth.")

**3. Results & Graph**
![Rule 31 Graph](images/rule31_result.png)
> **Observation:** Data loss occurs when the buffer size is smaller than the outstanding unacknowledged samples required for retransmission.

**4. Empirical Formula Derivation**
* Based on the results, the minimum depth must satisfy: $depth \ge \lceil RTT/PP \rceil + 2$.

---
### Rule 40
*.*

**1. Experimental Setup**
* **Network Condition:** (e.g., Simulated RTT using `tc`)
* **Publication Rate (PP):** (e.g., 20ms / 50Hz)
* **Variable:** History Depth ($1 \sim N$)

**2. Scenario**
(Describe how the test was performed, e.g., "Measuring message loss rate while increasing RTT and decreasing History Depth.")

**3. Results & Graph**
![Rule 31 Graph](images/rule31_result.png)
> **Observation:** Data loss occurs when the buffer size is smaller than the outstanding unacknowledged samples required for retransmission.

**4. Empirical Formula Derivation**
* Based on the results, the minimum depth must satisfy: $depth \ge \lceil RTT/PP \rceil + 2$.

---


## 📝 Notation Summary
* **mpi**: `max_samples_per_instance`
* **PP**: `Publish Period` (Time interval between consecutive samples)
* **RTT**: `Round Trip Time` (Network latency)
