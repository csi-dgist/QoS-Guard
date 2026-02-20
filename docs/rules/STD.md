# STD Rules

<style>
/* 1. 번호가 들어가는 칸(td) 설정 */
.md-typeset table td:first-child {
    /* 쏠림 방지 핵심: 셀 자체가 중앙 정렬의 기준이 됨 */
    text-align: center !important;
    vertical-align: middle !important;
    padding: 12px 0 !important;
    width: 50px !important;
    min-width: 50px !important;
}

/* 2. 숫자 배지 - 타원 방지 및 정중앙 배치 */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
    /* 타원 방지를 위해 inline-flex 사용 */
    display: inline-flex !important;
    align-items: center !important;
    justify-content: center !important;

    /* 정원 유지: 가로/세로를 완전히 똑같이 고정 */
    width: 26px !important;
    height: 26px !important;
    min-width: 26px !important;
    max-width: 26px !important; /* 가로로 늘어나는 타원 현상 차단 */
    
    /* 중앙 정렬 핵심: 상하 여백을 주고 좌우를 자동으로 설정 */
    margin: 4px auto !important; 
    
    border-radius: 50% !important;
    background-color: #f2f2f2 !important;
    
    /* 글자 스타일 */
    color: #000 !important;
    font-size: 13px !important;
    font-weight: bold !important;
    text-decoration: none !important;
    line-height: 1 !important;
    
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

