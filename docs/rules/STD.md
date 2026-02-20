# STD Rules

<style>
/* 1. 번호 칸(td) 내부의 모든 요소를 중앙으로 강제 배치 */
.md-typeset table td:first-child {
  padding: 0 !important;
  width: 50px !important;
  min-width: 50px !important;
  /* 핵심: 셀 내부에서 배지가 중앙에 오도록 함 */
  text-align: center !important;
  vertical-align: middle !important;
}

/* 2. 숫자 배지 (링크 유무 통합) */
.md-typeset table td:first-child,
.md-typeset table td:first-child a {
  /* inline-flex를 써야 정렬이 가장 안정적입니다 */
  display: inline-flex !important;
  align-items: center !important;
  justify-content: center !important;

  /* 수평 중앙 정렬을 보장하는 핵심 속성 */
  margin: 8px auto !important; 
  float: none !important;

  /* 원형 유지 설정 */
  width: 26px !important;
  height: 26px !important;
  min-width: 26px !important;
  min-height: 26px !important;
  border-radius: 50% !important;

  /* 스타일 */
  background-color: #f0f0f0 !important;
  color: #000 !important;
  font-size: 13px !important;
  font-weight: bold !important;
  text-decoration: none !important;
  transition: 0.2s;
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

