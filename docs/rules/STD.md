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
  display: block;
  
  /* 🔥 [해결] 클릭 시 브라우저/깃허브가 강제로 넣는 하늘색 테두리와 잔상 제거 */
  outline: none !important;
  box-shadow: none !important;
  -webkit-tap-highlight-color: transparent;
}

/* 브라우저 기본 화살표 및 클릭 시 하늘색 테두리 제거 */
.std-item summary {
  display: block;
  list-style: none;
  cursor: pointer;
  /* 🔥 [해결] 요소를 클릭했을 때 생기는 포커스 라인 제거 */
  outline: none !important;
}
.std-item summary::-webkit-details-marker {
  display: none;
}

/* 요소가 포커스되거나 열렸을 때 생기는 모든 하늘색 흔적 원천 차단 */
.std-item:focus, 
.std-item summary:focus, 
.std-item[open] {
  outline: none !important;
  box-shadow: none !important;
}

/* 호버 효과 (오직 마우스를 올렸을 때만 예쁜 보라색 포인트 작동) */
.std-item:hover {
  box-shadow: 0 8px 16px rgba(0,0,0,0.1) !important;
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

<hr class="hr-grad-left">

## Stage 1
*Intra-entity Dependency Validation*

<div class="std-list">

  <details class="std-item" id="rule-1">
    <summary>
      <div class="std-header">
        <span class="std-no">1</span>
        <span class="std-id">HIST ↔ RESLIM</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [HIST.kind = KEEP_LAST] ∧ [HIST.depth > RESLIM.mpi]
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub, Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, HistoryQosPolicy</b><br>
      "If history kind is KEEP_LAST, the depth must be less than or equal to max_samples_per_instance."
    </div>
  </details>

  <details class="std-item" id="rule-2">
    <summary>
      <div class="std-header">
        <span class="std-no">2</span>
        <span class="std-id">RESLIM ↔ RESLIM</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [max_samples < max_samples_per_instance]
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub, Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, ResourceLimitsQosPolicy</b><br>
      "The setting of max_samples must be consistent with max_samples_per_instance. It is required that max_samples >= max_samples_per_instance."
    </div>
  </details>

</div>

<hr class="hr-grad-left">

## Stage 2
*Inter-entity Dependency Validation*

<div class="std-list">

  <details class="std-item" id="rule-21">
    <summary>
      <div class="std-header">
        <span class="std-no">21</span>
        <span class="std-id">PART ↔ PART</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [Writer.PART ∩ Reader.PART] = ∅
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub ↔ Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, PartitionQosPolicy</b><br>
      "The PartitionQosPolicy allows the introduction of a logical partition... If the intersection of the set of partitions on the DataWriter and DataReader is empty, they will not communicate."
    </div>
  </details>

  <details class="std-item" id="rule-22">
    <summary>
      <div class="std-header">
        <span class="std-no">22</span>
        <span class="std-id">RELIAB ↔ RELIAB</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [Writer.RELIAB < Reader.RELIAB]
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub ↔ Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, ReliabilityQosPolicy</b><br>
      "This policy follows the Requested/Offered (RxO) contract. The Offered RELIABILITY kind of the DataWriter must be greater than or equal to the Requested RELIABILITY kind of the DataReader."
    </div>
  </details>

  <details class="std-item" id="rule-23">
    <summary>
      <div class="std-header">
        <span class="std-no">23</span>
        <span class="std-id">DURABL ↔ DURABL</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [Writer.DURABL < Reader.DURABL]
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub ↔ Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, DurabilityQosPolicy</b><br>
      "This policy follows the RxO contract. The Offered Durability kind of the DataWriter must be greater than or equal to the Requested Durability kind of the DataReader."
    </div>
  </details>

  <details class="std-item" id="rule-24">
    <summary>
      <div class="std-header">
        <span class="std-no">24</span>
        <span class="std-id">DEADLN ↔ DEADLN</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [Writer.DEADLN.period > Reader.DEADLN.period]
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub ↔ Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, DeadlineQosPolicy</b><br>
      "This policy follows the RxO contract. The Offered deadline period of the DataWriter must be less than or equal to the Requested deadline period of the DataReader."
    </div>
  </details>

  <details class="std-item" id="rule-25">
    <summary>
      <div class="std-header">
        <span class="std-no">25</span>
        <span class="std-id">LIVENS ↔ LIVENS</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [W.LIVENS < R.LIVENS] ∨ [W.lease > R.lease]
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub ↔ Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, LivelinessQosPolicy</b><br>
      "This policy follows the RxO contract. The Offered Liveliness kind must be greater than or equal to the Requested kind, and the writer's lease_duration must be less than or equal to the reader's."
    </div>
  </details>

  <details class="std-item" id="rule-26">
    <summary>
      <div class="std-header">
        <span class="std-no">26</span>
        <span class="std-id">OWNST ↔ OWNST</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [Writer.OWNST ≠ Reader.OWNST]
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub ↔ Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, OwnershipQosPolicy</b><br>
      "This policy follows the RxO contract. The DataWriter and DataReader must share the exact same Ownership kind setting (SHARED or EXCLUSIVE)."
    </div>
  </details>

  <details class="std-item" id="rule-27">
    <summary>
      <div class="std-header">
        <span class="std-no">27</span>
        <span class="std-id">DESTORD ↔ DESTORD</span>
        <span style="font-size: 0.8em; color: #999;">Structural</span>
      </div>
      <div class="std-condition">
        [Writer.DESTORD < Reader.DESTORD]
      </div>
      <div class="std-footer">
        <div class="std-tag"><b>Entity</b> Pub ↔ Sub</div>
        <div class="std-tag"><b>Basis</b> STD</div>
      </div>
    </summary>
    <div style="margin-top: 16px; font-size: 0.9em; color: #2c3e50; line-height: 1.6; cursor: default;">
      <b>📄 OMG DDS Specification v1.4 — Section 2.2.3, DestinationOrderQosPolicy</b><br>
      "This policy follows the RxO contract. The Offered DestinationOrder kind of the DataWriter must be greater than or equal to the Requested kind of the DataReader."
    </div>
  </details>

</div>
<hr class="hr-grad-left">
