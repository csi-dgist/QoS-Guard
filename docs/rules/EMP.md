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

### Rule 31: History Depth vs. Network Latency
*Justifies the dependency between History Depth and RTT in Reliable communication.*

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

### Rule 38: Deadline Period vs. Publish Period
*Validates the timing margin required for Deadline QoS to avoid false-positive violations.*



**1. Experimental Setup**
* **Entity:** Subscriber
* **Publish Period (PP):** (e.g., 10ms)
* **Deadline Period:** Adjusted from $1 \times PP$ to $3 \times PP$

**2. Scenario**
(e.g., "Monitoring `on_requested_deadline_missed` callbacks under jittery network conditions.")

**3. Results & Observation**
* **Jitter Analysis:** Even without network failure, jitter causes arrival times to exceed $1 \times PP$.
* **Safe Margin:** A margin of at least $2 \times PP$ is required to ensure stability.

---

### Rule 39: Liveliness Lease Duration vs. Publish Period
*Analyzes the relationship between Liveliness Lease and heartbeat frequency.*

**1. Scenario & Result**
(Fill in your specific experimental data for Rule 39 here.)

---

*(Note: Follow the same structure for Rules 6, 32, 33, 36, 37, and 40.)*

## 📝 Notation Summary
* **mpi**: `max_samples_per_instance`
* **PP**: `Publish Period` (Time interval between consecutive samples)
* **RTT**: `Round Trip Time` (Network latency)
