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

.std-reference {
  display: none;
  margin-top: 12px;
  padding-top: 12px;
  border-top: 1px solid #edf0f2;
  color: #4b5563;
  font-size: 0.9em;
  line-height: 1.5;
}

.std-item.is-open .std-reference {
  display: block;
}
</style>

This page describes the QoS dependency and consistency rules derived from the **OMG DDS** and **ROS 2 Standard** specifications. Violation of these rules typically results in entity creation failure or immediate communication incompatibility.

<hr class="hr-grad-left">

## Stage 1
*Intra-entity Dependency Validation*

<div class="std-list">

  <a href="#rule-1" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.18, HISTORY</b><br><br>
      The setting of HISTORY depth must be consistent with the RESOURCE_LIMITS max_samples_per_instance. <br>For these
two QoS to be consistent, they must verify that <b>depth <= max_samples_per_instance</b>.
    </div>
  </a>

  <a href="#rule-2" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.19, RESOURCE_LIMITS</b><br><br>
      The setting of RESOURCE_LIMITS max_samples must be consistent with the max_samples_per_instance. <br>For these
two values to be consistent they must verify that <b>max_samples >= max_samples_per_instance.</b>
    </div>
  </a>

</div>

<hr class="hr-grad-left">

## Stage 2
*Inter-entity Dependency Validation*

<div class="std-list">

  <a href="#rule-21" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.13, PARTITION</b><br><br>
      For a DataReader to see the changes made to an instance by a DataWriter, not only the Topic must match, but also <b>they
must share a common partition.</b>
    </div>
  </a>

  <a href="#rule-22" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.14, REALIABILITY</b><br>
      <br>The value offered is considered compatible with the value requested if and only if the inequality “<b>offered kind >=
requested kind</b>” evaluates to ‘TRUE.’ <br>For the purposes of this inequality, the values of RELIABILITY kind are
considered ordered such that <b>BEST_EFFORT < RELIABLE</b>
    </div>
  </a>

  <a href="#rule-23" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.4, DURABILITY</b><br>
      <br>The value offered is considered compatible with the value requested if and only if the inequality “<b>offered kind >=
requested kind</b> evaluates to ‘TRUE.’ <br>For the purposes of this inequality, the values of DURABILITY kind are considered
ordered such that <b>VOLATILE < TRANSIENT_LOCAL < TRANSIENT < PERSISTENT</b>.
    </div>
  </a>

  <a href="#rule-24" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.7, DEADLINE</b><br>
      <br>The value offered is considered compatible with the value requested if and only if the inequality “<b>offered deadline period
<= requested deadline period</b>” evaluates to ‘TRUE'.
    </div>
  </a>

  <a href="#rule-25" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.11, LIVELINESS</b><br>
      <br>1. the inequality “<b>offered kind >= requested kind</b>” evaluates to ‘TRUE.’ For the purposes of this inequality, the
values of LIVELINESS kind are considered ordered such that:
AUTOMATIC < MANUAL_BY_PARTICIPANT < MANUAL_BY_TOPIC.
<br>2. the inequality “<b>offered lease_duration <= requested lease_duration</b>” evaluates to TRUE.
    </div>
  </a>

  <a href="#rule-26" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.9, OWNERSHIP</b><br>
      <br>The value of the OWNERSHIP kind <b>offered must exactly match the one requested</b> or else they are considered.
incompat
    </div>
  </a>

  <a href="#rule-27" class="std-item">
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
    <div class="std-reference">
      <b>OMG DDS Specification v1.2 - Section 7.1.3.17, DESTINATION_ORDER</b><br>
      <br>The value offered is considered compatible with the value requested if and only if the inequality “<b>offered kind >=
requested kind</b>” evaluates to ‘TRUE.’ For the purposes of this inequality, the values of DESTINATION_ORDER kind are
considered ordered such that <b>BY_RECEPTION_TIMESTAMP < BY_SOURCE_TIMESTAMP</b>.
    </div>
  </a>

</div>
<hr class="hr-grad-left">

<script>
document.querySelectorAll('.std-item').forEach((item) => {
  item.addEventListener('click', (event) => {
    event.preventDefault();
    item.classList.toggle('is-open');
  });
});
</script>
