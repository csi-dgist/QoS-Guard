# STD Rules

<style>
  table {
    width: 100%;
    border-collapse: collapse;
    margin: 25px 0;
    font-size: 0.95em;
    font-family: 'Segoe UI', Tahoma, sans-serif;
    box-shadow: 0 0 20px rgba(0, 0, 0, 0.1);
    border-radius: 8px 8px 0 0;
    overflow: hidden;
  }

  table thead tr {
    background-color: #009879; /* DDS 표준 느낌의 진한 초록색 */
    color: #ffffff;
    text-align: center;
    font-weight: bold;
  }

  table th, table td {
    padding: 12px 15px;
    border: 1px solid #dddddd;
  }

  table tbody tr {
    border-bottom: 1px solid #dddddd;
  }

  table tbody tr:nth-of-type(even) {
    background-color: #f3f3f3; /* 줄바꿈 색상 구분 */
  }

  table tbody tr:last-of-type {
    border-bottom: 2px solid #009879;
  }

  /* 수식이 들어가는 세 번째 열(QoS Conflict Condition) 강조 */
  table td:nth-child(3) {
    font-family: 'Consolas', monospace;
    color: #c0392b;
    background-color: #fff9f9;
    font-weight: 500;
  }

  /* No. 열 너비 조절 및 중앙 정렬 */
  table td:nth-child(1) {
    text-align: center;
    font-weight: bold;
    width: 50px;
  }
</style>

This page describes the QoS dependency and consistency rules derived from the **OMG DDS** and **ROS 2 Standard** specifications. Violation of these rules typically results in entity creation failure or immediate communication incompatibility.

---

## Stage 1
*Intra-entity Dependency Validation*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| [1](#rule-1) | HIST ↔ RESLIM | $[HIST.kind = KEEP\_LAST] \wedge [HIST.depth > mpi]$ | Structural | Pub, Sub | STD |
| [2](#rule-2) | RESLIM ↔ RESLIM | $[max\_samples < max\_samples\_per\_instance]$ | Structural | Pub, Sub | STD |

---

## Stage 2
*Inter-entity Dependency Validation*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| [21](#rule-21) | PART ↔ PART | $[Writer.PART \cap Reader.PART] = \emptyset$ | Structural | Pub ↔ Sub | STD |
| [22](#rule-22) | RELIAB ↔ RELIAB | $[Writer.RELIAB < Reader.RELIAB]$ | Structural | Pub ↔ Sub | STD |
| [23](#rule-23) | DURABL ↔ DURABL | $[Writer.DURABL < Reader.DURABL]$ | Structural | Pub ↔ Sub | STD |
| [24](#rule-24) | DEADLN ↔ DEADLN | $[Writer.DEADLN.period > Reader.DEADLN.period]$ | Structural | Pub ↔ Sub | STD |
| [25](#rule-25) | LIVENS ↔ LIVENS | $[W.LIVENS < R.LIVENS] \vee [W.lease > R.lease]$ | Structural | Pub ↔ Sub | STD |
| [26](#rule-26) | OWNST ↔ OWNST | $[Writer.OWNST \neq Reader.OWNST]$ | Structural | Pub ↔ Sub | STD |
| [27](#rule-27) | DESTORD ↔ DESTORD | $[Writer.DESTORD < Reader.DESTORD]$ | Structural | Pub ↔ Sub | STD |

---

