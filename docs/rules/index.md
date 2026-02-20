# QoS Rules Overview

<style>
/* 1. 기본 폰트 및 배경 정리 */
.md-typeset {
    font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
}

/* 2. 표 레이아웃: 선을 없애고 여백 강조 */
.md-typeset table {
    border-collapse: separate;
    border-spacing: 0;
    border: 1px solid #f0f0f2;
    border-radius: 12px;
    overflow: hidden;
    margin: 2em 0;
    width: 100%;
    background: #ffffff;
}

/* 3. 헤더: 아주 연한 그레이 배경에 짙은 텍스트 (고급스러움) */
.md-typeset table thead {
    background-color: #f8f9fb;
}

.md-typeset table th {
    color: #475569 !important; /* 짙은 블루그레이 */
    font-weight: 600 !important;
    font-size: 12px !important;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    padding: 14px 16px !important;
    border-bottom: 1px solid #f0f0f2 !important;
}

/* 4. 첫 번째 열(No.): 강조하되 촌스럽지 않게 */
.md-typeset table td:first-child {
    font-family: 'JetBrains Mono', monospace;
    font-size: 14px !important;
    font-weight: 500 !important;
    color: #6366f1 !important; /* 세련된 인디고 블루 */
    text-align: center !important;
    background: #fcfcff;
    width: 60px;
}

/* 5. 수식 칸: 코드 스니펫 느낌 */
.md-typeset table td:nth-child(3) {
    font-family: 'Fira Code', 'Consolas', monospace;
    font-size: 13px;
    color: #1e293b;
    line-height: 1.6;
    background: #ffffff;
}

/* 6. 행 호버: 아주 은은하게 */
.md-typeset table tr {
    transition: background-color 0.1s ease;
}

.md-typeset table tr:hover {
    background-color: #fbfbfe !important;
}

.md-typeset table td {
    padding: 12px 16px !important;
    border-bottom: 1px solid #f0f0f2 !important;
    font-size: 13.5px;
}

/* 7. Basis 배지: 색감을 파스텔톤으로 톤다운 */
.basis-tag {
    display: inline-flex;
    align-items: center;
    padding: 2px 8px;
    border-radius: 6px;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.02em;
}
.basis-std { background-color: #ecfdf5; color: #059669; border: 1px solid #d1fae5; }
.basis-imp { background-color: #eff6ff; color: #2563eb; border: 1px solid #dbeafe; }
.basis-emp { background-color: #fff7ed; color: #ea580c; border: 1px solid #ffedd5; }

/* Stage 헤더 스타일 */
.stage-header {
    font-size: 1.5em;
    font-weight: 700;
    color: #1e293b;
    margin-top: 2.5em;
    display: flex;
    align-items: center;
    gap: 10px;
}

.stage-header::before {
    content: "";
    width: 4px;
    height: 24px;
    background: #6366f1;
    border-radius: 10px;
}
</style>

This section covers 40 dependency-violation rules classified into three stages. 

Choose a category from the sidebar to see detailed constraints.

## Full List of 40 Dependency-Violation Rules

We have identified and classified 40 rules that govern the relationships between ROS 2 QoS policies. 

These are implemented in **QoS Guard** for static verification. 

<hr class="hr-grad-left">

<h2 class="stage-header">Stage 1 | Intra-entity Dependency Validation</h2>
*Identifies internal conflicts by analyzing each entity's QoS profile independently.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| 1 | HIST ↔ RESLIM | $[HIST.kind = KEEP\_LAST] \wedge [HIST.depth > mpi]$ | Structural | Pub, Sub | <span class="basis-tag basis-std">STD</span> |
| 2 | RESLIM ↔ RESLIM | $[max\_samples < max\_samples\_per\_instance]$ | Structural | Pub, Sub | STD |
| 3 | RELIAB → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | IMP |
| 4 | RELIAB → OWNST | $[OWNST = EXCLUSIVE] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | <span class="basis-tag basis-imp">IMP</span> |
| 5 | RELIAB → LIVENS | $[LIVENS = MANUAL] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | IMP |
| 6 | LFSPAN → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [LFSPAN.duration > 0]$ | Functional | Pub | <span class="basis-tag basis-emp">EMP</span> |
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
| 19 | ENTFAC → DURABL | $[DURABL \neq VOLATILE] \wedge [autoenable = FALSE]$ | Operational | Pub, Sub | <span class="basis-tag basis-imp">IMP</span> |
| 20 | PART → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [PART.names \neq \emptyset]$ | Operational | Pub, Sub | IMP |

<hr class="hr-grad-left">

<h2 class="stage-header">Stage 2 | Inter-entity Dependency Validation</h2>
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

<hr class="hr-grad-left">

<h2 class="stage-header">Stage 3 | Timing-based Dependency Validation</h2>
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

<hr class="hr-grad-left">

!!! info "Notation Summary"
    * **mpi**: `max_samples_per_instance` 
    * **PP**: `Publish Period` 
    * **RTT**: `Round Trip Time`
