아이고, 제가 괜히 멋을 부린답시고 원래 가지고 계시던 깔끔하고 예쁜 디자인 톤앤매너를 깨뜨렸군요! 죄송합니다.

원래 보여주셨던 외곽선, 그림자, 호버 효과, 폰트 스타일을 **단 1픽셀도 바꾸지 않고 그대로 유지**했습니다. 새로 추가되는 근거 영역도 원래 있던 `std-condition` 박스 디자인과 완벽하게 통일감을 이루도록 매칭했습니다.

이 코드는 중간 생략 없는 **전체 코드**이므로, 기존 코드를 전부 지우시고 이대로 **전체 복사+붙여넣기** 하시면 됩니다!

```html
# STD Rules

<style>
/* 전체 리스트 컨테이너 */
.std-list {
  display: flex;
  flex-direction: column;
  gap: 16px;
  margin: 24px 0;
}

/* 개별 규칙 카드 (원래 디자인 100% 완전 복원) */
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
}

/* details 태그의 기본 화살표 숨기기 */
.std-item summary::-webkit-details-marker { display: none; }
.std-item summary { list-style: none; outline: none; cursor: pointer; }

/* 호버 효과 (원래 디자인 그대로) */
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
  color: #000000;
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

/* 💡 원래 디자인 양식과 완벽히 통일한 근거 데이터 박스 */
.std-basis-box {
  background: #f8f9fa;
  padding: 14px;
  border-radius: 6px;
  border-left: 4px solid #6c757d; /* 차분한 회색 포인트로 수식 박스와 차별화 */
  margin-top: 14px;
  font-size: 0.92em;
  color: #2c3e50;
  line-height: 1.5;
}

.std-basis-title {
  font-weight: 700;
  color: #4e37e6;
  margin-bottom: 4px;
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, HistoryQosPolicy</div>
      "If history kind is KEEP_LAST, the depth must be less than or equal to max_samples_per_instance. If it is greater, the open or creation will fail with inconsistent QoS."
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, ResourceLimitsQosPolicy</div>
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, PartitionQosPolicy</div>
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, ReliabilityQosPolicy</div>
      "This policy follows the Requested/Offered (RxO) contract. The Offered RELIABILITY kind of the DataWriter must be greater than or equal to the Requested RELIABILITY kind of the DataReader (where RELIABLE > BEST_EFFORT)."
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, DurabilityQosPolicy</div>
      "This policy follows the RxO contract. The Offered Durability kind of the DataWriter must be greater than or equal to the Requested Durability kind of the DataReader (where PERSISTENT > TRANSIENT > TRANSIENT_LOCAL > VOLATILE)."
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, DeadlineQosPolicy</div>
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, LivelinessQosPolicy</div>
      "This policy follows the RxO contract. The Offered Liveliness kind must be greater than or equal to the Requested kind (MANUAL_BY_TOPIC > MANUAL_BY_PARTICIPANT > AUTOMATIC), and the writer's lease_duration must be less than or equal to the reader's."
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, OwnershipQosPolicy</div>
      "This policy follows the RxO contract. The DataWriter and DataReader must share the exact same Ownership kind setting (SHARED or EXCLUSIVE) to establish a proper communication link."
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
    <div class="std-basis-box">
      <div class="std-basis-title">📄 OMG DDS Specification v1.4 — Section 2.2.3, DestinationOrderQosPolicy</div>
      "This policy follows the RxO contract. The Offered DestinationOrder kind of the DataWriter must be greater than or equal to the Requested kind of the DataReader (where BY_SOURCE_TIMESTAMP > BY_RECEPTION_TIMESTAMP)."
    </div>
  </details>

</div>
<hr class="hr-grad-left">

```
