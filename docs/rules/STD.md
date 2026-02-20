# STD Rules

<style>
.md-typeset table td:first-child {
  text-align: center;
  padding: 8px !important;
}

.md-typeset table td:first-child {
  color: #000;
  font-weight: bold;
  font-size: 13px;
}

.md-typeset table td:first-child, 
.md-typeset table td:first-child a {
  display: inline-block;
  width: 24px;         
  height: 24px;
  line-height: 24px;  
  border-radius: 50%;
  background-color: #f0f0f0;
  transition: 0.2s;
}

.md-typeset table td:first-child a {
  color: #000 !important;
  text-decoration: none !important;
}

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

