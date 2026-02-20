# STD Rules

<style>
/* 1. 번호가 들어가는 칸(td)을 정중앙 정렬 기계로 만들기 */
.md-typeset table td:first-child {
    display: flex !important;
    justify-content: center !important; /* 수평 중앙 */
    align-items: center !important;     /* 수직 중앙 */
    padding: 12px 0 !important;         /* 상하 여백만 유지 */
    width: 60px !important;             /* 칸 너비를 충분히 확보 */
    min-width: 60px !important;
    border: none !important;            /* 테두리 간섭 방지 */
}

/* 2. 숫자 배지 - 크기 고정 및 타원 방지 */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
    display: flex !important;
    justify-content: center !important;
    align-items: center !important;

    /* 원의 크기를 완전히 똑같이 고정하여 타원 방지 */
    width: 26px !important;
    height: 26px !important;
    min-width: 26px !important;
    max-width: 26px !important;
    
    margin: 0 !important;               /* 쏠림 방지를 위해 외부 마진 제거 */
    border-radius: 50% !important;      /* 무조건 정원 */
    background-color: #f2f2f2 !important;
    
    /* 글자 스타일 */
    color: #000 !important;
    font-size: 13px !important;
    font-weight: bold !important;
    text-decoration: none !important;
    line-height: 1 !important;          /* 텍스트 높이 간섭 제거 */
    
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

