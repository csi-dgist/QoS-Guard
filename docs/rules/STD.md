# STD Rules

<style>
/* 1. 번호가 들어가는 칸(td) 설정 */
.md-typeset table td:first-child {
    padding: 0 !important;
    width: 45px !important;
    min-width: 45px !important;
    /* 핵심: 셀 자체가 정렬을 주도하게 함 */
    text-align: center !important;
    vertical-align: middle !important;
}

/* 2. 숫자 배지 (가장 중요한 수정 부분) */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
    /* inline-flex로 변경하여 자기 크기를 고수하게 함 */
    display: inline-flex !important;
    align-items: center !important;
    justify-content: center !important;
    
    /* 가로/세로를 완전히 고정 */
    width: 26px !important;
    height: 26px !important;
    min-width: 26px !important; /* 최소 너비 강제 */
    min-height: 26px !important; /* 최소 높이 강제 */
    
    border-radius: 50% !important;
    margin: 5px auto !important; /* 상하 여백을 주어 중앙 배치 */
    
    background-color: #f2f2f2 !important;
    color: #000 !important;
    font-size: 13px !important;
    font-weight: bold !important;
    text-decoration: none !important;
    transition: 0.2s ease;
}

/* 3. 호버 시 강조 효과 */
.md-typeset table tr:hover td:first-child,
.md-typeset table tr:hover td:first-child a {
    background-color: #4e37e6 !important;
    color: #fff !important;
    transform: scale(1.1);
}
</style>

This page describes the QoS dependency and consistency rules derived from the **OMG DDS** and **ROS 2 Standard** specifications. Violation of these rules typically results in entity creation failure or immediate communication incompatibility.

---

## Stage 1
*Intra-entity Dependency Validation*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| 1 | HIST ↔ RESLIM | $[HIST.kind = KEEP\_LAST] \wedge [HIST.depth > mpi]$ | Structural | Pub, Sub | STD |
| 2 | RESLIM ↔ RESLIM | $[max\_samples < max\_samples\_per\_instance]$ | Structural | Pub, Sub | STD |

---

## Stage 2
*Inter-entity Dependency Validation*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| 21 | PART ↔ PART | $[Writer.PART \cap Reader.PART] = \emptyset$ | Structural | Pub ↔ Sub | STD |
| 22 | RELIAB ↔ RELIAB | $[Writer.RELIAB < Reader.RELIAB]$ | Structural | Pub ↔ Sub | STD |
| 23 | DURABL ↔ DURABL | $[Writer.DURABL < Reader.DURABL]$ | Structural | Pub ↔ Sub | STD |
| 24 | DEADLN ↔ DEADLN | $[Writer.DEADLN.period > Reader.DEADLN.period]$ | Structural | Pub ↔ Sub | STD |
| 25 | LIVENS ↔ LIVENS | $[W.LIVENS < R.LIVENS] \vee [W.lease > R.lease]$ | Structural | Pub ↔ Sub | STD |
| 26 | OWNST ↔ OWNST | $[Writer.OWNST \neq Reader.OWNST]$ | Structural | Pub ↔ Sub | STD |
| 27 | DESTORD ↔ DESTORD | $[Writer.DESTORD < Reader.DESTORD]$ | Structural | Pub ↔ Sub | STD |

---

