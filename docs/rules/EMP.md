# Stage 1 & 3: Empirical (EMP) Rules

This page describes the QoS dependency rules derived from **Empirical analysis and experimental results**. These rules focus on runtime performance, network conditions (e.g., RTT), and timing-critical dependencies that were validated through systematic testing.

---

## 🧪 Experimental Consistency (Stage 1)
*Internal dependencies within a single entity validated through empirical observation.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| 6 | LFSPAN → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [LFSPAN.duration > 0]$ | Functional | Pub | EMP |

---

## ⚡ Dynamic & Performance Rules (Stage 3)
*Rules focused on runtime environment, network latencies, and real-time constraints.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| 31 | HIST → RELIAB | $[RELIABLE] \wedge [KEEP\_LAST] \wedge [depth < \lceil RTT/PP \rceil + 2]$ | Functional | Pub | EMP |
| 32 | RESLIM → RELIAB | $[RELIABLE] \wedge [KEEP\_ALL] \wedge [mpi < \lceil RTT/PP \rceil + 1]$ | Functional | Pub | EMP |
| 33 | LFSPAN → RELIAB | $[RELIABLE] \wedge [LFSPAN.duration < RTT \times 2]$ | Functional | Pub | EMP |
| 36 | LIVENS → DEADLN | $[DEADLN.period > 0] \wedge [LIVENS.lease < DEADLN.period]$ | Functional | Sub | EMP |
| 37 | HIST → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [KEEP\_ALL] \wedge [mpi \ge default]$ | Operational | Pub | EMP |
| 38 | DEADLN → OWNST | $[OWNST = EXCLUSIVE] \wedge [DEADLN.period < 2 \times PP]$ | Operational | Sub | EMP |
| 39 | LIVENS → OWNST | $[OWNST = EXCLUSIVE] \wedge [lease < 2 \times PP]$ | Operational | Sub | EMP |
| 40 | DURABL → DEADLN | $[DEADLN.period > 0] \wedge [DURABL \ge TRAN\_LOCAL]$ | Operational | Sub | EMP |

---

!!! success "Importance of Empirical Rules"
    Empirical (EMP) rules are a core contribution of this research. While standard (STD) rules ensure that the system runs, EMP rules ensure that the system **performs reliably** under varying network conditions (RTT) and publication rates (PP).

*(Note: mpi = max_samples_per_instance, PP = Publish Period, RTT = Round Trip Time)*
