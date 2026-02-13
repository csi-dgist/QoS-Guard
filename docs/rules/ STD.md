# Stage 1: Structural Dependency Rules

이 단계에서는 단일 엔티티 내의 QoS 설정 간 구조적 모순을 검출합니다. (총 19개 규칙)

### 1. History vs Resource Limits
- **Rule**: `[HIST.kind = KEEP_LAST] ∧ [HIST.depth > RESLIM.max_samples_per_instance]`
- **설명**: 보관하려는 샘플 수(Depth)가 인스턴스당 할당된 최대 샘플 수보다 크면 설정 오류가 발생합니다.

### 2. Resource Limits Consistency
- **Rule**: `RESLIM.max_samples < RESLIM.max_samples_per_instance`
- **설명**: 전체 최대 샘플 수는 인스턴스당 최대 샘플 수보다 크거나 같아야 합니다.

### 3. Reliability vs Durability
- **Rule**: `[DURABL.kind ≥ TRANSIENT_LOCAL] ∧ [RELIAB.kind = BEST_EFFORT]`
- **설명**: 데이터를 보존(Durability)하려면 신뢰성 있는 전송(Reliable)이 전제되어야 합니다.
