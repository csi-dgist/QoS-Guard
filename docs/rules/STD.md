# STD Rules


<style>
/* 1. 번호 칸(td)을 중앙 정렬 기계로 개조 */
.md-typeset table td:first-child {
    /* 기존 레이아웃 무시하고 Flex 적용 */
    display: flex !important;
    justify-content: center !important; /* 수평 중앙 */
    align-items: center !important;     /* 수직 중앙 */
    
    /* 칸의 크기 고정 및 여백 제거 */
    min-width: 60px !important; 
    width: 60px !important;
    padding: 12px 0 !important; 
    border-right: 1px solid #eee; /* 구분선이 필요하면 유지 */
}

/* 2. 숫자 배지 (링크 유무 통합) */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
    /* 원 자체도 내부 숫자를 중앙에 맞춤 */
    display: inline-flex !important;
    justify-content: center !important;
    align-items: center !important;

    /* 타원 방지를 위해 가로/세로 완전 고정 */
    width: 26px !important;
    height: 26px !important;
    min-width: 26px !important;
    min-height: 26px !important;
    
    margin: 0 !important; /* 쏠림 방지를 위해 마진 제거 */
    border-radius: 50% !important;
    background-color: #f2f2f2 !important;
    
    /* 글자 스타일 */
    color: #000 !important;
    font-size: 13px !important;
    font-weight: bold !important;
    text-decoration: none !important;
    line-height: 1 !important; /* 행간 간섭 제거 */
    
    transition: 0.2s ease;
}

/* 3. 호버 시 효과 */
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

