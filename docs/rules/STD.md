# STD Rules

<style>
/* 1. 첫 번째 열(No.) 칸 자체를 중앙 정렬 기계로 만들기 */
.md-typeset table td:first-child {
    display: flex !important;
    justify-content: center !important; /* 수평 중앙 */
    align-items: center !important;     /* 수직 중앙 */
    padding: 10px 0 !important;         /* 상하 여백만 유지 */
    min-width: 50px !important;         /* 칸이 너무 좁아지지 않게 방어 */
    border: none !important;            /* 테두리 간섭 방지 */
}

/* 2. 원형 숫자 배지 */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
    display: flex !important;
    justify-content: center !important;
    align-items: center !important;
    
    width: 28px !important;             /* 원 크기 살짝 키움 */
    height: 28px !important;
    border-radius: 50% !important;      /* 무조건 정원 */
    
    background-color: #f2f2f2 !important;
    color: #000 !important;
    font-size: 13px !important;
    font-weight: bold !important;
    text-decoration: none !important;
    transition: background 0.2s, transform 0.2s;
}

/* 3. 호버 효과 */
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

