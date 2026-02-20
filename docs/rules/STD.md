# STD Rules


<style>
/* 전체 리스트 컨테이너 */
.std-list {
  display: flex;
  flex-direction: column;
  gap: 16px;
  margin: 24px 0;
}

/* 개별 규칙 카드 */
.std-item {
  border: 1px solid #e1e4e8;
  border-radius: 10px;
  background: #ffffff;
  padding: 16px 20px;
  transition: all 0.3s cubic-bezier(0.25, 0.8, 0.25, 1);
  position: relative;
  text-decoration: none !important;
  color: inherit !important;
}

/* 호버 효과: 그림자와 보라색 포인트 (사용자 선호 색상 반영) */
.std-item:hover {
  box-shadow: 0 8px 16px rgba(0,0,0,0.1);
  border-color: #4e37e6;
  transform: translateY(-2px);
}

/* 상단 라인: 번호와 타이틀 */
.std-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
}

.std-no {
  background: #f0f0f0;
  color: #444;
  width: 28px;
  height: 28px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 50%;
  font-weight: 800;
  font-size: 13px;
  transition: 0.3s;
}

.std-item:hover .std-no {
  background: #4e37e6;
  color: #fff;
}

.std-id {
  font-weight: 700;
  font-size: 1.05em;
  color: #2c3e50;
  flex-grow: 1;
  margin-left: 12px;
}

/* 위반 조건 박스 (수식 강조) */
.std-condition {
  background: #f8f9fa;
  padding: 14px;
  border-radius: 6px;
  font-family: 'Consolas', 'Monaco', monospace;
  border-left: 4px solid #4e37e6;
  margin: 8px 0;
  font-size: 0.95em;
  color: #000000; /* 수식 강조 색상 */
  line-height: 1.5;
}

/* 하단 메타 데이터 */
.std-footer {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
  margin-top: 10px;
  font-size: 0.82em;
}

.std-tag {
  display: flex;
  align-items: center;
  background: #f1f3f5;
  padding: 2px 8px;
  border-radius: 4px;
  color: #666;
}

.std-tag b {
  color: #4e37e6;
  margin-right: 5px;
  font-weight: 600;
}
</style>

This page describes the QoS dependency and consistency rules derived from the **OMG DDS** and **ROS 2 Standard** specifications. Violation of these rules typically results in entity creation failure or immediate communication incompatibility.

---

## Stage 1
*Intra-entity Dependency Validation*




<div class="std-list">

  <div class="std-item" id="rule-1">
    <div class="std-header">
      <span class="std-no">1</span>
      <span class="std-id">HIST ↔ RESLIM</span>
      <span style="font-size: 0.8em; color: #999;">Structural</span>
    </div>
    <div class="std-condition">
      [HIST.kind = KEEP\_LAST] \wedge [HIST.depth > mpi]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Pub, Sub</div>
      <div class="std-tag"><b>Basis</b> STD</div>
    </div>
  </div>

  <div class="std-item" id="rule-2">
    <div class="std-header">
      <span class="std-no">2</span>
      <span class="std-id">RESLIM ↔ RESLIM</span>
      <span style="font-size: 0.8em; color: #999;">Structural</span>
    </div>
    <div class="std-condition">
      [max\_samples < max\_samples\_per\_instance]
    </div>
    <div class="std-footer">
      <div class="std-tag"><b>Entity</b> Pub, Sub</div>
      <div class="std-tag"><b>Basis</b> STD</div>
    </div>
  </div>

</div>
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

