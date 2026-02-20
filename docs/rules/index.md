# QoS Rules Overview

<style>
.md-typeset table td:first-child {
    font-size: 1.2em !important;    /* ← 글자 크기 (1.5em, 18px 등으로 자유롭게 조절) */
    font-weight: 800 !important;    /* ← 더 굵게 (700~900 사이 선택) */
    text-align: center !important;  /* ← 가운데 정렬 */
    color: #333 !important;         /* ← 글자 색상 (필요시 변경) */
    vertical-align: middle !important;
}
</style>

This section covers 40+ dependency-violation rules classified into three stages. 
Choose a category from the sidebar to see detailed constraints.


## Full List of 40 Dependency-Violation Rules

We have identified and classified 40 rules that govern the relationships between ROS 2 QoS policies. 
These are implemented in **QoS Guard** for static verification.


## Stage 1 | Intra-entity Dependency Validation
*Identifies internal conflicts by analyzing each entity's QoS profile independently.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| 1 | HIST ↔ RESLIM | $[HIST.kind = KEEP\_LAST] \wedge [HIST.depth > mpi]$ | Structural | Pub, Sub | STD |
| 2 | RESLIM ↔ RESLIM | $[max\_samples < max\_samples\_per\_instance]$ | Structural | Pub, Sub | STD |
| 3 | RELIAB → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | IMP |
| 4 | RELIAB → OWNST | $[OWNST = EXCLUSIVE] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | IMP |
| 5 | RELIAB → LIVENS | $[LIVENS = MANUAL] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | IMP |
| 6 | LFSPAN → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [LFSPAN.duration > 0]$ | Functional | Pub | EMP |
| 7 | LFSPAN → DEADLN | $LFSPAN.duration < DEADLN.period$ | Functional | Sub | IMP |
| 8 | HIST → DESTORD | $[DESTORD = BY\_SOURCE] \wedge [HIST.kind = KEEP\_LAST] \wedge [depth = 1]$ | Functional | Sub | IMP |
| 9 | RESLIM → DESTORD | $[DESTORD = BY\_SOURCE] \wedge [KEEP\_ALL] \wedge [mpi = 1]$ | Functional | Sub | IMP |
| 10 | DEADLN → OWNST | $[OWNST = EXCLUSIVE] \wedge [DEADLN.period = \infty]$ | Functional | Sub | IMP |
| 11 | LIVENS → OWNST | $[OWNST = EXCLUSIVE] \wedge [LIVENS.lease = \infty]$ | Functional | Sub | IMP |
| 12 | LIVENS → RDLIFE | $[autopurge\_nowriter > 0] \wedge [LIVENS.lease = \infty]$ | Functional | Sub | IMP |
| 13 | RDLIFE → DURABL | $[DURABL \ge TRANSIENT] \wedge [autopurge\_disposed \neq \infty]$ | Functional | Sub | IMP |
| 14 | PART → DEADLN | $[DEADLN.period > 0] \wedge [PART.names \neq \emptyset]$ | Functional | Sub | IMP |
| 15 | PART → LIVENS | $[LIVENS = MANUAL] \wedge [PART.names \neq \emptyset]$ | Functional | Sub | IMP |
| 16 | OWNST → WDLIFE | $[autodispose = TRUE] \wedge [OWNST = EXCLUSIVE]$ | Functional | Sub | IMP |
| 17 | HIST → LFSPAN | $[HIST.KEEP\_LAST] \wedge [LFSPAN.duration > HIST.depth \times PP]$ | Operational | Pub, Sub | IMP |
| 18 | RESLIM → LFSPAN | $[KEEP\_ALL] \wedge [LFSPAN.duration > mpi \times PP]$ | Operational | Pub, Sub | IMP |
| 19 | ENTFAC → DURABL | $[DURABL \neq VOLATILE] \wedge [autoenable = FALSE]$ | Operational | Pub, Sub | IMP |
| 20 | PART → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [PART.names \neq \emptyset]$ | Operational | Pub, Sub | IMP |

---

## Stage 2 | Inter-entity Dependency Validation
*Prevents connection failures by checking RxO compatibility between Publisher and Subscriber pairs.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| 21 | PART ↔ PART | $[Writer.PART \cap Reader.PART] = \emptyset$ | Structural | Pub ↔ Sub | STD |
| 22 | RELIAB ↔ RELIAB | $[Writer.RELIAB < Reader.RELIAB]$ | Structural | Pub ↔ Sub | STD |
| 23 | DURABL ↔ DURABL | $[Writer.DURABL < Reader.DURABL]$ | Structural | Pub ↔ Sub | STD |
| 24 | DEADLN ↔ DEADLN | $[Writer.DEADLN.period > Reader.DEADLN.period]$ | Structural | Pub ↔ Sub | STD |
| 25 | LIVENS ↔ LIVENS | $[W.LIVENS < R.LIVENS] \vee [W.lease > R.lease]$ | Structural | Pub ↔ Sub | STD |
| 26 | OWNST ↔ OWNST | $[Writer.OWNST \neq Reader.OWNST]$ | Structural | Pub ↔ Sub | STD |
| 27 | DESTORD ↔ DESTORD | $[Writer.DESTORD < Reader.DESTORD]$ | Structural | Pub ↔ Sub | STD |
| 28 | WDLIFE → RDLIFE | $[W.autodispose = FALSE] \wedge [R.autopurge\_nowriter = 0]$ | Functional | Pub ↔ Sub | IMP |
| 29 | WDLIFE → RDLIFE | $[W.autodispose = FALSE] \wedge [R.autopurge\_disposed > 0]$ | Operational | Pub ↔ Sub | IMP |
| 30 | WDLIFE → RDLIFE | $[W.autodispose = FALSE] \wedge [R.autopurge\_nowriter = \infty]$ | Operational | Pub ↔ Sub | IMP |

---

## Stage 3 | Timing-based Dependency Validation
*Evaluates operational risks by integrating network parameters like RTT and publish period.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| 31 | HIST → RELIAB | $[RELIABLE] \wedge [KEEP\_LAST] \wedge [depth < \lceil RTT/PP \rceil + 2]$ | Functional | Pub | EMP |
| 32 | RESLIM → RELIAB | $[RELIABLE] \wedge [KEEP\_ALL] \wedge [mpi < \lceil RTT/PP \rceil + 1]$ | Functional | Pub | EMP |
| 33 | LFSPAN → RELIAB | $[RELIABLE] \wedge [LFSPAN.duration < RTT \times 2]$ | Functional | Pub | EMP |
| 34 | RELIAB → WDLIFE | $[autodispose = TRUE] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub | IMP |
| 35 | RELIAB → DEADLN | $[DEADLN.period > 0] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub ↔ Sub | IMP |
| 36 | LIVENS → DEADLN | $[DEADLN.period > 0] \wedge [LIVENS.lease < DEADLN.period]$ | Functional | Sub | EMP |
| 37 | HIST → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [KEEP\_ALL] \wedge [mpi \ge default]$ | Operational | Pub | EMP |
| 38 | DEADLN → OWNST | $[OWNST = EXCLUSIVE] \wedge [DEADLN.period < 2 \times PP]$ | Operational | Sub | EMP |
| 39 | LIVENS → OWNST | $[OWNST = EXCLUSIVE] \wedge [lease < 2 \times PP]$ | Operational | Sub | EMP |
| 40 | DURABL → DEADLN | $[DEADLN.period > 0] \wedge [DURABL \ge TRAN\_LOCAL]$ | Operational | Sub | EMP |

---
*(Note: mpi = max_samples_per_instance, PP = Publish Period, RTT = Round Trip Time)*
