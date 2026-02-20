# QoS Rules Overview

<style>
/* 1. 전체 폰트 스택 정돈 */
.md-typeset {
    font-family: 'Inter', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
}

/* 2. 표 디자인: 테두리는 최소화, 라운딩과 그림자로 부드럽게 */
.md-typeset table {
    border-collapse: separate;
    border-spacing: 0;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    overflow: hidden;
    margin: 24px 0;
    width: 100%;
    background: #ffffff;
    box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.05);
}

/* 3. 헤더: #4E5EB4 배경에 화이트 텍스트 */
.md-typeset table thead {
    background-color: #4E5EB4;
}

.md-typeset table th {
    color: #ffffff !important;
    font-weight: 600 !important;
    font-size: 13px !important;
    text-transform: uppercase;
    letter-spacing: 0.03em;
    padding: 14px 16px !important;
    border: none !important;
}

/* 4. 첫 번째 열(No.): 포인트 컬러 강조 */
.md-typeset table td:first-child {
    font-family: 'JetBrains Mono', monospace;
    font-size: 15px !important;
    font-weight: 600 !important;
    color: #4E5EB4 !important;
    text-align: center !important;
    background: #f8fafc; /* 매우 연한 그레이 블루 */
    width: 65px;
}

/* 5. 데이터 셀: 깔끔한 가독성 */
.md-typeset table td {
    padding: 14px 16px !important;
    border-bottom: 1px solid #f1f5f9 !important;
    font-size: 13.5px;
    color: #334155;
    vertical-align: middle;
}

/* 6. 수식(Violation Condition) 칸 스타일 */
.md-typeset table td:nth-child(3) {
    font-family: 'Fira Code', 'Consolas', monospace;
    font-size: 12.5px;
    color: #1e293b;
    background: #ffffff;
}

/* 7. 행 호버: 부드러운 전환 */
.md-typeset table tr:hover {
    background-color: #f1f5f9 !important;
}

/* 8. Basis 배지: #4E5EB4와 어울리는 파스텔 톤 */
.basis-tag {
    display: inline-flex;
    align-items: center;
    padding: 2px 8px;
    border-radius: 4px;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.02em;
}
/* STD: Emerald / IMP: Sky / EMP: Amber */
.basis-std { background-color: #ecfdf5; color: #059669; border: 1px solid #d1fae5; }
.basis-imp { background-color: #f0f9ff; color: #0369a1; border: 1px solid #e0f2fe; }
.basis-emp { background-color: #fffbeb; color: #b45309; border: 1px solid #fef3c7; }

/* Stage 헤더: 왼쪽 바 포인트 */
.stage-header {
    font-size: 1.4em;
    font-weight: 700;
    color: #1e293b;
    margin-top: 2em;
    padding-left: 14px;
    border-left: 4px solid #4E5EB4;
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
