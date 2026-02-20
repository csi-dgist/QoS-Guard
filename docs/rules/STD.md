# STD Rules

<style>
/* 1. 번호가 들어가는 칸(td) 설정 */
.md-typeset table td:first-child {
    text-align: center !important;    /* 수평 중앙 정렬 */
    vertical-align: middle !important;  /* 수직 중앙 정렬 */
    padding: 8px !important;
    width: 40px !important;           /* 칸 너비 고정 */
    min-width: 40px !important;
}

/* 2. 숫자 배지 - 링크(a)가 있든 없든 동일하게 적용 */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
    /* EMP.md 방식처럼 글자처럼 취급하되 크기 고정 */
    display: inline-block !important; 
    width: 24px !important;
    height: 24px !important;
    line-height: 24px !important;     /* 글자를 원의 수직 중앙으로 */
    
    /* 타원 방지를 위해 최대 너비 제한 */
    max-width: 24px !important;
    
    border-radius: 50% !important;
    background-color: #f0f0f0 !important;
    
    /* 글자 스타일 */
    color: #000 !important;
    text-decoration: none !important;
    font-weight: bold !important;
    font-size: 13px !important;
    transition: 0.2s;
}

/* 3. 호버 시 효과 (마우스를 올렸을 때) */
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

