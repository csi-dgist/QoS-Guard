# STD Rules

<style>
/* 1. 번호가 들어가는 칸(td) 설정 */
.md-typeset table td:first-child {
  text-align: center !important; /* 내부 요소를 수평 중앙으로 */
  vertical-align: middle !important; /* 수직 중앙 */
  padding: 10px 0 !important;
  width: 50px !important;
  min-width: 50px !important;
}

/* 2. 숫자 배지 (링크 유무 상관없이 적용) */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
  /* 핵심: inline-flex와 margin auto의 조합 */
  display: inline-flex !important;
  align-items: center !important;
  justify-content: center !important;
  
  /* 원형을 유지하기 위한 절대 고정 수치 */
  width: 26px !important;
  height: 26px !important;
  min-width: 26px !important;
  min-height: 26px !important;
  
  /* 왼쪽 쏠림 해결: 좌우 마진을 auto로 주면 정중앙에 고정됩니다 */
  margin-left: auto !important;
  margin-right: auto !important;
  margin-top: 4px !important;
  margin-bottom: 4px !important;

  border-radius: 50% !important;
  background-color: #f2f2f2 !important;
  color: #000 !important;
  font-size: 13px !important;
  font-weight: bold !important;
  text-decoration: none !important;
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

