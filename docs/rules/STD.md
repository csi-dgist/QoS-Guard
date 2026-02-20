# STD Rules

<style>
/* 1. 번호가 들어가는 칸(td) 설정 */
.md-typeset table td:first-child {
    padding: 0 !important;       /* 칸 안의 불필요한 여백 완전 제거 */
    width: 45px !important;      /* 칸 너비를 원보다 조금 넓게 고정 */
    min-width: 45px !important;
    vertical-align: middle !important; /* 수직 중앙 정렬 기본값 */
}

/* 2. 숫자 배지 (링크가 있든 없든 동일 적용) */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
    display: flex !important;    /* flex를 써야 정중앙 배치가 됨 */
    align-items: center !important;
    justify-content: center !important;
    
    margin: auto !important;     /* 칸 내에서 정중앙으로 밀어넣음 */
    width: 26px !important;      /* 원 크기 (줄이려면 이 값을 수정) */
    height: 26px !important;
    border-radius: 50% !important;
    
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
    background-color: #4e37e6 !important; /* 파란색 강조 */
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

