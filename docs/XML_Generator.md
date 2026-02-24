# DDS QoS XML Generator

DDS QoS 테스트 케이스를 위한 XML 파일 자동 생성 도구입니다. 16개 파라미터의 조합을 Pairwise 테스트 방식으로 생성하여 효율적인 테스트 케이스를 만듭니다.

## 목차

- [파라미터 목록](#파라미터-목록)
- [Pairwise 테스트 케이스 생성 방식](#pairwise-테스트-케이스-생성-방식)
- [최종 케이스 개수 산출](#최종-케이스-개수-산출)
- [실행 방법](#실행-방법)

---

## 파라미터 목록

### 전체 파라미터 개요

| 파라미터 | pub 값 개수 | pub 값 종류 | sub 값 개수 | sub 값 종류 | 제약 조건 | 최종 조합 개수 |
|---------|------------|------------|------------|------------|----------|---------------|
| **ENTITY_FACTORY** | 2 | True, False | 2 | True, False | pub=sub | **2개** |
| **DATA_EXISTS** | 2 | exists, not_exists | 2 | exists, not_exists | pub=sub | **2개** |
| **RELIABILITY** | 2 | RELIABLE, BEST_EFFORT | 2 | RELIABLE, BEST_EFFORT | 독립 | **4개** |
| **DURABILITY** | 4 | TRANSIENT_LOCAL, VOLATILE, TRANSIENT, PERSISTENT | 4 | TRANSIENT_LOCAL, VOLATILE, TRANSIENT, PERSISTENT | 독립 | **16개** |
| **DEADLINE** | 3 | PP, 2×PP, DURATION_INFINITY | 3 | PP, 2×PP, DURATION_INFINITY | 독립 | **9개** |
| **LIVELINESS** | 6 | (AUTOMATIC, 0.5×PP), (AUTOMATIC, PP), (AUTOMATIC, 2×PP), (AUTOMATIC, ∞), (MANUAL_BY_PARTICIPANT, None), (MANUAL_BY_TOPIC, None) | 6 | (AUTOMATIC, 0.5×PP), (AUTOMATIC, PP), (AUTOMATIC, 2×PP), (AUTOMATIC, ∞), (MANUAL_BY_PARTICIPANT, None), (MANUAL_BY_TOPIC, None) | 독립 | **36개** |
| **HISTORY** | 5 | (KEEP_ALL, None), (KEEP_LAST, 1), (KEEP_LAST, <(RTT/PP)+2), (KEEP_LAST, =(RTT/PP)+2), (KEEP_LAST, >(RTT/PP)+2) | 5 | (KEEP_ALL, None), (KEEP_LAST, 1), (KEEP_LAST, <(RTT/PP)+2), (KEEP_LAST, =(RTT/PP)+2), (KEEP_LAST, >(RTT/PP)+2) | 독립 | **25개** |
| **RESOURCE_LIMITS_MAX_SAMPLES_PER_INSTANCE** | 4 | 1, <(RTT/PP)+2, =(RTT/PP)+2, >(RTT/PP)+2 | 4 | 1, <(RTT/PP)+2, =(RTT/PP)+2, >(RTT/PP)+2 | 독립 | **16개** |
| **RESOURCE_LIMITS_MAX_SAMPLES** | 2 | 1, (RTT/PP+3)×PP | 2 | 1, (RTT/PP+3)×PP | 독립 | **4개** |
| **LIFESPAN** | 2 | 0.5×RTT, DURATION_INFINITY | 2 | 0.5×RTT, DURATION_INFINITY | 독립 | **4개** |
| **OWNERSHIP** | 2 | SHARED, EXCLUSIVE | 2 | SHARED, EXCLUSIVE | 독립 | **4개** |
| **DESTINATION_ORDER** | 2 | BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP | 2 | BY_RECEPTION_TIMESTAMP, BY_SOURCE_TIMESTAMP | 독립 | **4개** |
| **WRITER_DATA_LIFECYCLE** | 2 | True, False | 0 | None (사용 안 함) | pub만 | **2개** |
| **READER_DATA_LIFECYCLE_NO_WRITER** | 0 | None (사용 안 함) | 2 | 0, 3 | sub만 | **2개** |
| **READER_DATA_LIFECYCLE_DISPOSED** | 0 | None (사용 안 함) | 2 | 0, 3 | sub만 | **2개** |

### 파라미터별 상세 설명

#### 1. ENTITY_FACTORY
- **제약**: pub와 sub는 항상 동일한 값이어야 함
- **조합**: (True, True), (False, False)

#### 2. DATA_EXISTS
- **설명**: PARTITION, USER_DATA, GROUP_DATA, TOPIC_DATA를 통합한 파라미터
- **제약**: pub와 sub는 항상 동일한 값이어야 함
- **조합**: (exists, exists), (not_exists, not_exists)

#### 3. RELIABILITY
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 2 × 2 = 4개

#### 4. DURABILITY
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 4 × 4 = 16개

#### 5. DEADLINE
- **값 종류**: 
  - `PP`: D < 2×PP 케이스
  - `2×PP`: D ≥ 2×PP 케이스
  - `DURATION_INFINITY`: 무한대
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 3 × 3 = 9개

#### 6. LIVELINESS
- **값 종류**: 
  - `(AUTOMATIC, 0.5×PP)`: lease_duration < 2×PP
  - `(AUTOMATIC, PP)`: lease_duration < 2×PP
  - `(AUTOMATIC, 2×PP)`: lease_duration ≥ 2×PP
  - `(AUTOMATIC, DURATION_INFINITY)`: 무한대
  - `(MANUAL_BY_PARTICIPANT, None)`: MANUAL 종류
  - `(MANUAL_BY_TOPIC, None)`: MANUAL 종류
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 6 × 6 = 36개

#### 7. HISTORY
- **값 종류**: 
  - `(KEEP_ALL, None)`: KEEP_ALL
  - `(KEEP_LAST, 1)`: depth=1
  - `(KEEP_LAST, <(RTT/PP)+2)`: depth < (RTT/PP)+2
  - `(KEEP_LAST, =(RTT/PP)+2)`: depth = (RTT/PP)+2
  - `(KEEP_LAST, >(RTT/PP)+2)`: depth > (RTT/PP)+2
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 5 × 5 = 25개

#### 8. RESOURCE_LIMITS_MAX_SAMPLES_PER_INSTANCE
- **값 종류**: 
  - `1`: = 1
  - `<(RTT/PP)+2`: < (RTT/PP)+2
  - `=(RTT/PP)+2`: = (RTT/PP)+2
  - `>(RTT/PP)+2`: > (RTT/PP)+2
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 4 × 4 = 16개

#### 9. RESOURCE_LIMITS_MAX_SAMPLES
- **값 종류**: `1`, `(RTT/PP+3)×PP`
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 2 × 2 = 4개

#### 10. LIFESPAN
- **값 종류**: `0.5×RTT`, `DURATION_INFINITY`
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 2 × 2 = 4개

#### 11. OWNERSHIP
- **값 종류**: `SHARED`, `EXCLUSIVE`
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 2 × 2 = 4개

#### 12. DESTINATION_ORDER
- **값 종류**: `BY_RECEPTION_TIMESTAMP`, `BY_SOURCE_TIMESTAMP`
- **독립 설정**: pub와 sub를 독립적으로 설정 가능
- **조합**: 2 × 2 = 4개

#### 13. WRITER_DATA_LIFECYCLE
- **설명**: Publisher에만 적용되는 파라미터
- **값 종류**: `True`, `False`
- **조합**: 2개

#### 14. READER_DATA_LIFECYCLE_NO_WRITER
- **설명**: Subscriber에만 적용되는 파라미터
- **값 종류**: `0`, `3`
- **조합**: 2개

#### 15. READER_DATA_LIFECYCLE_DISPOSED
- **설명**: Subscriber에만 적용되는 파라미터
- **값 종류**: `0`, `3`
- **제약**: READER_DATA_LIFECYCLE_NO_WRITER와 동일한 값이어야 함
- **조합**: 2개

---

## Pairwise 테스트 케이스 생성 방식

### Pairwise 테스트란?

Pairwise 테스트는 모든 파라미터 쌍의 조합을 최소 한 번씩 커버하는 최소 테스트 케이스를 생성하는 방법입니다.

### 생성 과정

1. **파라미터 수집**
   - 16개 파라미터 중 pub 또는 sub 값이 있는 파라미터만 선택

2. **각 파라미터의 조합 생성**
   - `_get_param_values()` 함수를 통해 각 파라미터의 pub/sub 조합 생성
   - 제약 조건에 따라 조합 개수가 결정됨

3. **Pairwise 알고리즘 적용**
   - **allpairspy 라이브러리 사용** (설치된 경우): 최적화된 알고리즘
   - **자체 알고리즘** (allpairspy 없을 때): 간단한 Pairwise 구현
     - 모든 파라미터 쌍 생성: C(16,2) = 120개 쌍
     - 각 쌍의 값 조합 생성
     - 테스트 케이스 생성 (기본값 + 현재 쌍 값)
     - 중복 제거

4. **제약 조건 적용**
   - READER_DATA_LIFECYCLE_NO_WRITER와 READER_DATA_LIFECYCLE_DISPOSED는 동일한 값이어야 함
   - 서로 다른 값이면 해당 조합 제외

### 예시

**파라미터 A, B, C가 있고 각각 2개, 3개, 2개의 값이 있다면:**

- **전체 조합**: 2 × 3 × 2 = 12개
- **Pairwise 케이스**: 모든 쌍 (A-B, A-C, B-C)을 최소 한 번씩 커버하는 최소 케이스 (약 6-8개)

이렇게 하면 전체 조합 대비 훨씬 적은 수의 테스트 케이스로도 효과적인 테스트 커버리지를 확보할 수 있습니다.

---

## 최종 케이스 개수 산출

### 이론적 전체 조합 개수

만약 모든 파라미터를 독립적으로 조합한다면:

```
2 × 2 × 4 × 16 × 9 × 36 × 25 × 16 × 4 × 4 × 4 × 4 × 2 × 2 × 2
= 약 271조 개
```

### 실제 생성되는 케이스 개수

Pairwise 알고리즘을 사용하면:

- **실제 생성 개수**: 약 **853개** (PP=0.1, RTT=0.2 기준)
- **효율성**: 전체 조합 대비 약 **0.0000003%**

### 개수 산출 방법

1. 각 파라미터의 조합 개수 계산
   - 독립 파라미터: `pub 개수 × sub 개수`
   - 제약 파라미터: `pub 개수` (pub=sub 제한) 또는 `sub 개수` (pub만/sub만)

2. Pairwise 알고리즘 적용
   - 모든 파라미터 쌍을 최소 한 번씩 커버하는 최소 케이스 생성
   - 실제 개수는 파라미터 간 상호작용에 따라 결정됨

3. 제약 조건 필터링
   - READER_DATA_LIFECYCLE 제약 조건에 맞지 않는 케이스 제외

### 실제 실행 결과 (PP=0.1, RTT=0.2)

```
총 853개의 테스트 케이스 생성
XML 파일: 1,706개 (853개 케이스 × 2 파일)
  - pub_qos_case_00001.xml ~ pub_qos_case_00853.xml
  - sub_qos_case_00001.xml ~ sub_qos_case_00853.xml
```

---

## 실행 방법

### 1. 필수 요구사항

- Python 3.6 이상
- 필수 라이브러리:
  ```bash
  pip install xml
  ```

- 선택적 라이브러리 (최적화된 Pairwise 알고리즘):
  ```bash
  pip install allpairspy
  ```
  > **참고**: allpairspy가 없어도 자체 알고리즘으로 동작합니다.

### 2. 파일 구조

```
XML Generator/
├── xml_generator.py      # 메인 코드
├── xml/
│   ├── pub_qos.xml       # Publisher 템플릿
│   └── sub_qos.xml       # Subscriber 템플릿
└── output/               # 생성된 XML 파일 저장 디렉토리
```

### 3. 실행 방법

#### 방법 1: 직접 실행

```bash
cd "/home/csi/ros2_ws/src/XML Generator"
python3 xml_generator.py
```

실행 시 다음 정보를 입력합니다:
- **PP (Publication Period)**: Publication Period 값 (초)
- **RTT (Round Trip Time)**: Round Trip Time 값 (초)

#### 방법 2: 코드에서 직접 호출

```python
from xml_generator import XMLGenerator

# XMLGenerator 생성
generator = XMLGenerator(pp=0.1, rtt=0.2)

# 테스트 케이스 생성
generator.generate_all_test_cases(output_dir='output')
```

#### 방법 3: 자동 입력 (스크립트)

```bash
cd "/home/csi/ros2_ws/src/XML Generator"
python3 xml_generator.py <<< $'0.1\n0.2\n'
```

### 4. 실행 예시

```
============================================================
DDS QoS XML Generator
============================================================
PP (Publication Period) 값을 입력하세요 (초): 0.1
RTT (Round Trip Time) 값을 입력하세요 (초): 0.2

입력된 값:
  PP: 0.1 초
  RTT: 0.2 초
  RTT/PP: 2.00
테스트 케이스 조합 생성 중...
총 853개의 테스트 케이스 생성

XML 파일 생성 중...
진행 중: 100/853 (11%)
진행 중: 200/853 (23%)
...
진행 중: 800/853 (93%)

완료! 총 853개의 테스트 케이스가 생성되었습니다.
출력 디렉토리: output/
```

### 5. 출력 파일

생성된 XML 파일은 `output/` 디렉토리에 저장됩니다:

- **Publisher XML**: `pub_qos_case_00001.xml` ~ `pub_qos_case_00853.xml`
- **Subscriber XML**: `sub_qos_case_00001.xml` ~ `sub_qos_case_00853.xml`

각 XML 파일은 DDS QoS 프로파일을 포함하며, 테스트 케이스에 맞는 파라미터 값이 설정되어 있습니다.

### 6. 주의사항

- **PP 값**: 0보다 큰 값이어야 합니다.
- **기존 파일**: 같은 이름의 파일이 있으면 덮어씁니다.
- **디렉토리**: `output/` 디렉토리가 없으면 자동으로 생성됩니다.

---

## 참고사항

### 동적 값 계산

일부 파라미터 값은 PP와 RTT에 따라 동적으로 계산됩니다:

- **DEADLINE**: `PP`, `2×PP`
- **LIVELINESS**: `0.5×PP`, `PP`, `2×PP`
- **HISTORY**: `(RTT/PP)+2` 기반 계산
- **RESOURCE_LIMITS**: `(RTT/PP)+2` 기반 계산
- **LIFESPAN**: `0.5×RTT`

따라서 PP와 RTT 값에 따라 생성되는 케이스 개수와 값이 달라질 수 있습니다.

### 제약 조건

1. **ENTITY_FACTORY, DATA_EXISTS**: pub와 sub는 항상 동일한 값
2. **READER_DATA_LIFECYCLE**: NO_WRITER와 DISPOSED는 동일한 값이어야 함
3. **WRITER_DATA_LIFECYCLE**: pub만 사용 (sub는 None)
4. **READER_DATA_LIFECYCLE**: sub만 사용 (pub는 None)

---

## 라이선스

이 프로젝트는 DDS QoS 테스트 케이스 생성을 위한 도구입니다.
