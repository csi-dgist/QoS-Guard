# QoS Rules Overview

This section covers 40+ dependency-violation rules classified into three stages. 
Choose a category from the sidebar to see detailed constraints.

!! info
    Our analysis covers Discovery, Data Exchange, and Disassociation phases.
## 📋 Full List of 40 Dependency-Violation Rules

We have identified and classified 40 rules that govern the relationships between ROS 2 QoS policies. These are implemented in **QoS Guard** for static verification.

# QoS Rules Overview

This page provides a comprehensive list of the 40 dependency-violation rules identified in our research. The rules are categorized into three stages based on their verification context.

---

## Stage 1: Structural & Intra-entity Rules
*Focuses on individual entity (Publisher or Subscriber) settings and internal consistency.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity |
|:---:|:---|:---|:---:|:---:|
| 1 | HIST ↔ RESLIM | $[HIST.kind = KEEP\_LAST] \wedge [HIST.depth > RESLIM.mpi]$ | Structural | Pub, Sub |
| 2 | RESLIM ↔ RESLIM | $[RESLIM.max\_samples < RESLIM.max\_samples\_per\_instance]$ | Structural | Pub, Sub |
| 3 | HIST → DESTORD | $[DESTORD = BY\_SOURCE] \wedge [HIST.kind = KEEP\_LAST] \wedge [depth = 1]$ | Functional | Sub |
| 4 | RESLIM → DESTORD | $[DESTORD = BY\_SOURCE] \wedge [HIST.kind = KEEP\_ALL] \wedge [mpi = 1]$ | Functional | Sub |
| 5 | RDLIFE → DURABL | $[DURABL \ge TRANSIENT] \wedge [RDLIFE.autopurge\_delay = 0]$ | Operational | Sub |
| 6 | ENTFAC → DURABL | $[DURABL = VOLATILE] \wedge [ENTFAC.autoenable = FALSE]$ | Operational | Pub, Sub |
| 7 | PART → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [PARTITION \neq \emptyset]$ | Operational | Pub, Sub |
| 8 | PART → DEADLN | $[DEADLN.period > 0] \wedge [PARTITION \neq \emptyset]$ | Operational | Pub, Sub |
| 9 | PART → LIVENS | $[LIVENS = MANUAL\_TOPIC] \wedge [PARTITION \neq \emptyset]$ | Operational | Sub |
| 10 | OWNST → WDLIFE | $[WDLIFE.autodispose = TRUE] \wedge [OWNST = EXCLUSIVE]$ | Operational | Pub |
| 11 | HIST → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [KEEP\_LAST] \wedge [depth < \lceil RTT/PP \rceil + 2]$ | Functional | Pub |
| 12 | RESLIM → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [KEEP\_ALL] \wedge [mpi < \lceil RTT/PP \rceil + 2]$ | Functional | Pub |
| 13 | LFSPAN → DURABL | $[HIST.kind = KEEP\_LAST] \wedge [LFSPAN.duration < RTT]$ | Functional | Pub |
| 14 | HIST ↔ LFSPAN | $[DURABL \ge TRAN\_LOCAL] \wedge [LFSPAN.duration > HIST.depth \times PP]$ | Functional | Pub |
| 15 | RESLIM ↔ LFSPAN | $[HIST.kind = KEEP\_ALL] \wedge [LFSPAN.duration > RESLIM.mpi \times PP]$ | Functional | Pub |
| 16 | DEADLN → OWNST | $[OWNST = EXCLUSIVE] \wedge [DEADLN.period = \infty]$ | Functional | Sub |
| 17 | LIVENS → OWNST | $[OWNST = EXCLUSIVE] \wedge [LIVENS.lease\_duration = \infty]$ | Functional | Sub |
| 18 | LIVENS → RDLIFE | $[RDLIFE.autopurge\_delay > 0] \wedge [LIVENS.lease\_duration = \infty]$ | Operational | Sub |
| 19 | RELIAB → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub |
| 20 | LFSPAN → DEADLN | $[LFSPAN.duration < DEADLN.period]$ | Structural | Pub, Sub |

---

## Stage 2: RxO (Required-versus-Offered) Rules
*Focuses on compatibility and matching between Publishers and Subscribers.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity |
|:---:|:---|:---|:---:|:---:|
| 21 | PART ↔ PART | $[Pub.PARTITION \cap Sub.PARTITION = \emptyset]$ | Structural | Pub ↔ Sub |
| 22 | RELIAB ↔ RELIAB | $[Pub.RELIAB < Sub.RELIAB]$ | Structural | Pub ↔ Sub |
| 23 | DURABL ↔ DURABL | $[Pub.DURABL < Sub.DURABL]$ | Structural | Pub ↔ Sub |
| 24 | DEADLN ↔ DEADLN | $[Pub.DEADLN.period > Sub.DEADLN.period]$ | Structural | Pub ↔ Sub |
| 25 | LIVENS ↔ LIVENS | $[Pub.LIVENS.kind < Sub.LIVENS.kind] \vee [Pub.LIVENS.lease > Sub.LIVENS.lease]$ | Structural | Pub ↔ Sub |
| 26 | OWNST ↔ OWNST | $[Pub.OWNST \neq Sub.OWNST]$ | Structural | Pub ↔ Sub |
| 27 | DESTORD ↔ DESTORD | $[Pub.DESTORD < Sub.DESTORD]$ | Structural | Pub ↔ Sub |
| 28 | WDLIFE → RDLIFE | $[WDLIFE.autodispose = FALSE] \wedge [RDLIFE.autopurge\_delay > 0]$ | Functional | Pub ↔ Sub |

---

## Stage 3: Dynamic & Performance Rules
*Focuses on runtime environment dependencies, network conditions, and timing.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity |
|:---:|:---|:---|:---:|:---:|
| 29 | HIST → RELIAB | $[RELIAB = RELIABLE] \wedge [KEEP\_LAST] \wedge [depth < \lceil RTT/PP \rceil + 2]$ | Functional | Pub |
| 30 | RESLIM → RELIAB | $[RELIAB = RELIABLE] \wedge [KEEP\_ALL] \wedge [mpi < \lceil RTT/PP \rceil + 2]$ | Functional | Pub |
| 31 | LFSPAN → RELIAB | $[RELIAB = RELIABLE] \wedge [LFSPAN.duration < RTT]$ | Functional | Pub |
| 32 | RELIAB → OWNST | $[OWNST = EXCLUSIVE] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub ↔ Sub |
| 33 | RELIAB → DEADLN | $[DEADLN.period > 0] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub ↔ Sub |
| 34 | LIVENS → DEADLN | $[DEADLN.period > 0] \wedge [LIVENS.lease\_duration < DEADLN.period]$ | Functional | Sub |
| 35 | RELIAB → LIVENS | $[LIVENS = MANUAL\_TOPIC] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub ↔ Sub |
| 36 | DEADLN → OWNST | $[OWNST = EXCLUSIVE] \wedge [DEADLN.period < 2 \times PP]$ | Functional | Sub |
| 37 | LIVENS → OWNST | $[OWNST = EXCLUSIVE] \wedge [LIVENS.lease\_duration < 2 \times PP]$ | Functional | Sub |
| 38 | RELIAB → WDLIFE | $[WDLIFE.autodispose = TRUE] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub |
| 39 | HIST → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [KEEP\_LAST] \wedge [depth > \lceil RTT/PP \rceil + 2]$ | Operational | Pub |
| 40 | DURABL → DEADLN | $[DEADLN.period > 0] \wedge [DURABL \ge TRAN\_LOCAL]$ | Operational | Sub |

---

!!! info "Acronyms"
    - **mpi**: max_samples_per_instance
    - **PP**: Publish Period
    - **RTT**: Round Trip Time
