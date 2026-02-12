# QoS Guard: ROS 2 DDS QoS 정책 종속성 분석 및 정적 검증 도구

<p align="center">
  <img alt="ROS2 logo" src="https://img.shields.io/badge/ROS--2-Humble%20%7C%20Jazzy%20%7C%20Kilted-blue?style=for-the-badge">
  <img alt="Fast DDS logo" src="https://img.shields.io/badge/Fast--DDS-2.6.9%20%7C%202.14.x-brightgreen?style=for-the-badge">
  <img alt="RTI Connext logo" src="https://img.shields.io/badge/RTI%20Connext-6.0.1%20%7C%207.3.0-orange?style=for-the-badge">
  <img alt="Cyclone DDS logo" src="https://img.shields.io/badge/Cyclone%20DDS-All%20Versions-lightgrey?style=for-the-badge">
  <img alt="Python logo" src="https://img.shields.io/badge/Python-3.10+-blue?style=for-the-badge">
</p>

## 📝 프로젝트 개요

ROS 2는 Data Distribution Service(DDS)를 기반으로 구축되며, 통신 가용성, 신뢰성, 자원 사용량을 제어하기 위해 20개 이상의 Quality of Service(QoS) 정책을 활용합니다. 하지만 실제 사용 환경에서 사용자들은 이러한 정책들을 결합하는 명확한 가이드나 사전 검증 절차가 부족하여, 시행착오 기반 튜닝이나 예기치 않은 런타임 오류가 발생하는 경우가 많습니다.

이러한 문제를 해결하기 위해 본 프로젝트는 DDS 퍼블리셔-서브스크라이버 통신을 Discovery(발견), Data Exchange(데이터 교환), Disassociation(연결 해제)의 3단계로 분해하고, 16개 핵심 QoS 정책이 각 단계에서 어떻게 동작하는지 튜토리얼 형식으로 설명합니다. 또한 정책 간 상호 종속성을 체계적으로 분석하여 QoS 종속성 체인(QoS Dependency Chain)을 도출하고, 40가지 일반적 제약 조건을 종속성 위반 규칙(Dependency-Violation Rules)으로 분류했습니다.

이 분석을 기반으로 **QoS Guard 패키지**를 개발했습니다. 이 도구는 배포 전에 **ROS 2 패키지를 오프라인으로 검증**하여 잠재적 충돌을 미리 감지할 수 있습니다. 사용자가 ROS 2 세션을 실행하지 않고도 안전하게 QoS 설정을 구성할 수 있게 됩니다.

개념적 통찰력과 실용적인 도구를 모두 제공함으로써, 이 작업은 ROS 2 사용자가 QoS 정책을 더 잘 이해하고 관리하도록 도와 최종적으로 로봇 통신의 신뢰성과 자원 활용의 효율성을 향상시킵니다.

## 💡 실행 방법

이 도구는 **ROS 2 패키지** 또는 **독립형 Python 스크립트**로 실행할 수 있습니다. ROS 2 런타임이 필요하지 않으며, 표준 Python 라이브러리만 사용합니다.

### 실행 모드

1. **패키지 모드** (기본): ROS 2 패키지 경로를 지정하여 내부 QoS XML 파일 자동 스캔 및 검증  
2. **XML 페어 모드** (옵션): `--xml` 옵션과 함께 pub.xml, sub.xml 두 파일을 직접 지정하여 검증
3. **목록 모드** (옵션): `--list` 옵션과 함께 패키지 경로 내 모든 XML 파일 목록 출력

### 실행 인자

- `package_path`: ROS 2 패키지 경로 (패키지 모드, 기본)
- `--xml`: XML 페어 모드 활성화, pub.xml sub.xml 파일 지정 필요
- `--list`: 목록 모드 활성화
- `pub.xml` / `sub.xml`: Writer/Reader QoS 프로파일 경로 (XML 페어 모드 시)
- `dds`: DDS 벤더 – `fast` | `cyclone` | `connext`
- `ros_version`: ROS 2 버전 – `humble` | `jazzy` | `kilted`
- `publish_period`: Writer의 메시지 발행 주기(PP), 예: `40ms` (선택, 기본값 40ms)
- `rtt`: 예상 왕복 지연 시간(RTT), 예: `50ms` (선택, 기본값 50ms)

## 💡 DDS 벤더별 지원 현황

| **DDS 벤더** | **XML 프로파일 지원** | **소스 코드 스캔** | **특이사항** |
| --- | --- | --- | --- |
| **Fast DDS** | ✓ | ✓ | XML과 소스 코드 QoS 설정 병합 분석 지원 |
| **RTI Connext** | **✓** | **✓** | **XML 파서 및 프로파일 매칭 완벽 지원** |
| **Cyclone DDS** | **N/A** | ✓ | **설계상 XML QoS를 지원하지 않음** (코드 스캔 전용) |

> ⚠️ **Cyclone DDS 안내**: Cyclone DDS는 미들웨어 설계 사양상 별도의 XML QoS 프로파일 형식을 제공하지 않습니다. 따라서 본 도구는 Cyclone DDS 사용 시 **소스 코드 내의 QoS 설정(`rclcpp::QoS` 등)을 직접 파싱**하여 분석을 수행합니다.
>

## 💡 DDS 벤더 및 버전별 지원 매트릭스

본 도구는 각 DDS 벤더와 ROS 2 버전별로 상이한 QoS 인터페이스 지원 현황을 다음과 같이 추적하여 검증에 반영합니다.

| **QoSPolicy** | **ROS 2 Interface** | **Humble Fast(2.6.9)** | **Jazzy Fast(2.14.0)** | **Kilted Fast(2.14.4)** | **Cyclone (0.10.5)** | **Connext (6.0.1)** |
| --- | --- | --- | --- | --- | --- | --- |
| **ENTITY_FACTORY** | X | △ | O | O | X | O |
| **PARTITION** | X | O | O | O | X | O |
| **USER_DATA** | X | O | O | O | X | O |
| **GROUP_DATA** | X | O | O | O | X | O |
| **TOPIC_DATA** | X | O | O | O | X | O |
| **RELIABILITY** | O | O | O | O | O | O |
| **DURABILITY** | O | O | O | O | O | O |
| **DEADLINE** | O | O | O | O | O | O |
| **LIVELINESS** | O | O | O | O | O | O |
| **LEASE_DURATION** | O | O | O | O | O | O |
| **HISTORY** | O | O | O | O | O | O |
| **DEPTH** | O | O | O | O | O | O |
| **RESOURCE_LIMITS** | X | O | O | O | X | O |
| **LIFESPAN** | O | O | O | O | O | O |
| **OWNERSHIP** | X | △ | O | O | X | O |
| **OWNERSHIP STRENGTH** | X | △ | O | O | X | O |
| **DESTINATION_ORDER** | X | X | X | X | X | O |
| **WRITER_DATA_LIFECYCLE** | X | X | O | O | X | O |
| **READER_DATA_LIFECYCLE** | X | X | X | X | X | O |

> **범례:** O (지원), X (미지원), △ (부분 지원/제한적 지원)
>

### 외부 XML 프로파일 (환경 변수)

Fast DDS 전용 기능입니다. 패키지 모드에서 `dds=fast`로 실행할 때, 다음 환경 변수로 지정된 **외부 XML 파일**도 함께 스캔하여 시스템 전반의 QoS 설정을 종합적으로 분석합니다.

| 환경 변수 | 설명 | 용도 |
| --- | --- | --- |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | Fast DDS 기본 프로파일 XML 파일 경로 | 시스템 기본 QoS 설정 |
| `RMW_FASTRTPS_CONFIG_FILE` | ROS 2 rmw_fastrtps용 XML 설정 파일 경로 | ROS 2 미들웨어 QoS 설정 |

외부 XML 경로는 사용자 환경마다 다르므로, **사용자가 직접 환경 변수를 설정**해야 합니다. 환경 변수가 설정되지 않은 경우, 패키지 내부의 XML 파일만 스캔합니다.

```bash
# 실행 전 환경 변수 설정 예시
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/default_profiles.xml
qos_guard /path/to/package fast humble

# 또는 명령어와 함께 한 번만 적용
FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/default_profiles.xml qos_guard /path/to/package fast humble
```

---

## 🔧 설치 및 실행 방법

### 방법 A: ROS 2 패키지로 설치 및 실행

```bash
# 1. ROS 2 워크스페이스 생성 (기존에 없는 경우)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. 저장소 클론
git clone --branch QosGuard_v3 https://github.com/QosGuard-Anonymous/qos-guard.github.io.git

# 3. 패키지 빌드
cd ~/ros2_ws
colcon build --packages-select qos_guard
source install/setup.bash

# 4. 실행 예제
ros2 run qos_guard qos_guard /path/to/ros2_package                    # 패키지 모드 (기본값)
ros2 run qos_guard qos_guard /path/to/ros2_package fast jazzy        # 패키지 모드 (특정 DDS/ROS)
ros2 run qos_guard qos_guard --xml pub.xml sub.xml fast humble         # XML 페어 모드
ros2 run qos_guard qos_guard --list /path/to/package                 # XML 파일 목록 출력
```

### 방법 B: 독립형 Python 스크립트로 실행

```bash
# 1. 저장소 클론
git clone --branch QosGuard_v3 https://github.com/QosGuard-Anonymous/qos-guard.github.io.git
cd qos-guard.github.io/qos_guard

# 2. Python으로 직접 실행 (Python 3.10+ 필요)
python3 -m qos_guard.qos_checker /path/to/ros2_package                    # 패키지 모드 (기본값)
python3 -m qos_guard.qos_checker /path/to/ros2_package fast jazzy        # 패키지 모드 (특정 DDS/ROS)
python3 -m qos_guard.qos_checker --xml pub.xml sub.xml fast humble         # XML 페어 모드
python3 -m qos_guard.qos_checker --list /path/to/package                 # XML 파일 목록 출력
```

**중요**: ROS 2가 설치되어 있지 않아도 Python 3.10+만 있으면 실행 가능합니다.

---

## 📂 프로젝트 구조

```
qos_guard/
├── qos_guard/                    # 메인 Python 패키지
│   ├── __init__.py               # 패키지 초기화
│   ├── cli.py                    # CLI 인자 파싱 및 명령어 처리
│   ├── xml_parser.py             # XML QoS 프로파일 파싱 모듈
│   ├── rules_fastdds_humble.py   # Fast DDS + Humble용 규칙 검사 엔진
│   ├── package_scanner.py        # ROS 2 패키지 QoS XML 자동 스캐너
│   ├── output.py                 # 검증 결과 출력 포맷터
│   └── qos_checker.py            # 메인 진입점 및 조정 로직
├── resource/                     # 리소스 파일 디렉토리
│   └── qos_guard                 # 추가 리소스
├── test/                         # 단위 테스트 디렉토리
│   ├── test_copyright.py         # 저작권 테스트
│   ├── test_flake8.py            # 코드 스타일 테스트
│   └── test_pep257.py            # 문서화 테스트
├── test_xml/                     # 테스트용 XML 프로파일
│   ├── pub.xml                   # Writer QoS 프로파일 예제
│   └── sub.xml                   # Reader QoS 프로파일 예제
├── package.xml                   # ROS 2 패키지 메타데이터
├── setup.cfg                     # 패키지 설정
└── setup.py                      # Python 패키지 설치 스크립트
```

---

## 🧪 주요 기능

이 도구는 다음과 같은 QoS 설정을 파싱하고 검증합니다:

### 지원하는 QoS 정책
- `ENTITY_FACTORY`, `PARTITION`, `USER_DATA`, `GROUP_DATA`, `TOPIC_DATA`
- `RELIABILITY`, `DURABILITY`, `DEADLINE`, `LIVELINESS`
- `HISTORY`, `RESOURCE_LIMITS`, `LIFESPAN`
- `OWNERSHIP(+STRENGTH)`, `DESTINATION_ORDER`
- `WRITER_DATA_LIFECYCLE`, `READER_DATA_LIFECYCLE`


## 📝 QoS-Guard: Fast DDS 프로파일 우선순위 가이드 (Humble ~ Kilted)

이 프로젝트는 **ROS 2 Humble, Jazzy, Kilted** 버전에서 Fast DDS(rmw_fastrtps)를 사용할 때 발생하는 QoS 설정 충돌을 방지하기 위해 다음의 우선순위 규칙을 준수합니다.

### 🔝 QoS 적용 계층 구조 (Hierarchy)

여러 위치에 QoS가 정의되어 있을 경우, 아래 순서에 따라 **가장 높은 우선순위의 설정이 하위 설정을 완전히 덮어씁니다 (Override).**

| 우선순위 | 설정 위치 | 매칭 방식 | 비고 |
| --- | --- | --- | --- |
| **1 (최우선)** | **ROS 2 소스 코드 (`rclcpp`)** | `rclcpp::QoS` (non-DEFAULT) | XML 설정을 완전히 무시 |
| **2** | **XML: `<topic profile_name="...">`** | 토픽 이름 자동 매칭 | [강력 권장] 버전 무관 최우선 XML 레이어 |
| **3** | **XML: `<data_writer>` / `<data_reader>`** | 토픽 이름 자동 매칭 | Jazzy/Kilted 스타일 |
| **4** | **XML: `<publisher>` / `<subscriber>`** | 코드 내 명시적 이름 지정 | Humble 스타일 (인라인 `<topic>` 포함) |
| **5 (최하위)** | **XML: `is_default_profile="true"`** | Fallback (기본값) | 명시적 설정이 없을 때만 적용 |

---

### 💡 버전별 특징 및 매칭 방식

* **베스트 프랙티스:** 버전에 관계없이 항상 우선 적용되는 설정을 하려면 **2순위 (`<topic>`)**를 사용하십시오.
* **Humble:** 주로 4순위인 `<publisher profile_name="my_pub">` 방식을 사용하며, 이를 적용하려면 코드에서 `PublisherOptions`를 통해 이름을 명시적으로 매칭해야 합니다.
* **Jazzy/Kilted:** 3순위인 `<data_writer profile_name="/topic_name">` 방식이 도입되어 토픽 이름만으로 자동 매칭이 가능해졌습니다.
---

### 🔍 실제 적용된 QoS 확인 방법

설정한 QoS가 실제로 반영되었는지 확인하려면 다음 명령어를 사용하세요.

```bash
# 특정 토픽의 실제 적용된 QoS 상세 확인
ros2 topic echo /your_topic_name --qos-profile all
```

---

## 📎 패키지 모드: QoS 프로파일 매칭 규칙

패키지 모드(`qos_guard /path/to/pkg fast humble`)에서 pub/sub XML 파일을 **어떻게 쌍(pair)으로 묶는지** 정의합니다.

### 1. Base 이름 추출 (정규식)

`profile_name`에서 `_pub`, `_subscriber`, `_writer`, `_reader`, `_profile` 등의 접미사를 제거한 **순수 기본 이름**을 base로 사용합니다.

| profile_name | 추출된 base |
| --- | --- |
| `cmd_vel_pub` | `cmd_vel` |
| `cmd_vel_subscriber` | `cmd_vel` |
| `latency_publisher_profile` | `latency` |
| `datawriter_profile_example` | `datawriter_profile_example` |

→ `cmd_vel_pub`와 `cmd_vel_subscriber`는 **같은 base**로 인식되어 서로 매칭됩니다.

### 2. 매칭 규칙 상세

| 조건 | 동작 방식 |
| --- | --- |
| **base가 동일** | 해당 pub ↔ sub만 쌍으로 생성하여 검증 |
| **profile_name 없음** | 모든 pub × sub 조합 생성하여 전체 검증 |
| **Wildcard 키워드 포함** | `default`, `common`, `generic` 포함 시 → **모든 pub × sub 조합** 생성 |

### 3. Wildcard 키워드 확장

`generic_qos_pub`, `default_profile_sub`, `common_publisher`처럼 base에 다음 키워드가 포함되면 **모든 조합과 매칭**됩니다.

- `default`
- `common` 
- `generic`

이를 통해 범용 프로파일을 여러 토픽과 함께 검사할 수 있습니다.

### 4. 매칭 알고리즘 요약

```
1. profile_name에서 base 추출 (정규식으로 _pub, _sub 등 접미사 제거)
2. 추출된 base에 default/common/generic 포함? → 모든 조합 생성 (Wildcard)
3. 그 외: base가 같은 pub-sub만 매칭
4. profile_name 없음? → 모든 조합 생성
```

---


## 🧪 검증 결과 분류 체계

Writer와 Reader 프로파일에 대해 **40개 이상의 종속성 규칙 위반**을 검증하고 시스템에 미치는 영향을 기준으로 분류합니다.

- **<span style="color:red">Category A: Structural Violation (구조적 위반)</span>**
    - 위반 시 RMW 수준에서 매칭이 실패하거나 시스템의 비정상적 종료(Segfault 등)를 야기함.
- **<span style="color:orange">Category B: Functional Violation (기능적 위반)</span>**
    - 통신은 성립되나, Reliable 보장이나 Persistence 유지 등 사용자가 의도한 데이터 품질 규약이 깨짐.
- **<span style="color:purple">Category C: Operational Violation (성능적 위반)</span>**
    - 기능적 결함은 없으나 불필요한 메모리 점유, 대역폭 낭비 등 자원 사용의 비효율성을 초래함.

## 📋 QoS Guard Rules (Stage별 정렬)

QoS Guard가 오프라인 검증 시 사용하는 40가지 핵심 규칙 리스트입니다.

| **No.** | **Identifier** | **QoS Conflict Condition** | **Entity Scope** | **Stage** | **Dependency** |
| --- | --- | --- | --- | --- | --- |
| 1 | **HIST ↔ RESLIM** | $[HIST.kind = KEEP\_LAST] \land [HIST.depth > mpi]$ | Pub, Sub | **1** | <span style="color:red">Structural</span> |
| 2 | **RESLIM ↔ RESLIM** | $[max\_samples < max\_samples\_per\_instance]$ | Pub, Sub | **1** | <span style="color:red">Structural</span> |
| 3 | **LFSPAN → DEADLN** | $LFSPAN.duration < DEADLN.period$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 4 | **HIST → DESTORD** | $[DESTORD=BY\_SOURCE] \land [HIST.kind=KEEP\_LAST] \land [depth=1]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 5 | **RESLIM → DESTORD** | $[DESTORD=BY\_SOURCE] \land [KEEP\_ALL] \land [mpi=1]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 6 | **LFSPAN → DURABL** | $[DURABL \ge TRAN\_LOCAL] \land [LFSPAN.duration > 0]$ | Pub | **1** | <span style="color:orange">Functional</span> |
| 7 | **HIST ↔ LFSPAN** | $[HIST.KEEP\_LAST] \land [LFSPAN.duration > HIST.depth \times PP]$ | Pub, Sub | **1** | <span style="color:purple">Operational</span> |
| 8 | **RESLIM ↔ LFSPAN** | $[KEEP\_ALL] \land [LFSPAN.duration > mpi \times PP]$ | Pub, Sub | **1** | <span style="color:purple">Operational</span> |
| 9 | **DEADLN → OWNST** | $[OWNST=EXCLUSIVE] \land [DEADLN.period = \infty]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 10 | **LIVENS → OWNST** | $[OWNST=EXCLUSIVE] \land [LIVENS.lease = \infty]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 11 | **LIVENS → RDLIFE** | $[autopurge\_nowriter > 0] \land [LIVENS.lease = \infty]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 12 | **RDLIFE → DURABL** | $[DURABL \ge TRANSIENT] \land [autopurge\_disposed \neq \infty]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 13 | **ENTFAC → DURABL** | $[DURABL=VOLATILE] \land [autoenable = FALSE]$ | Pub, Sub | **1** | <span style="color:purple">Operational</span> |
| 14 | **PART → DURABL** | $[DURABL \ge TRAN\_LOCAL] \land [PART.names \neq \emptyset]$ | Pub, Sub | **1** | <span style="color:purple">Operational</span> |
| 15 | **PART → DEADLN** | $[DEADLN.period > 0] \land [PART.names \neq \emptyset]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 16 | **PART → LIVENS** | $[LIVENS=MANUAL] \land [PART.names \neq \emptyset]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 17 | **OWNST → WDLIFE** | $[autodispose=TRUE] \land [OWNST=EXCLUSIVE]$ | Sub | **1** | <span style="color:orange">Functional</span> |
| 28 | **RELIAB → DURABL** | $[DURABL \ge TRAN\_LOCAL] \land [RELIAB=BEST\_EFFORT]$ | Pub, Sub | **1** | <span style="color:orange">Functional</span> |
| 32 | **RELIAB → OWNST** | $[OWNST=EXCLUSIVE] \land [RELIAB=BEST\_EFFORT]$ | Pub, Sub | **1** | <span style="color:orange">Functional</span> |
| 35 | **RELIAB → LIVENS** | $[LIVENS=MANUAL] \land [RELIAB=BEST\_EFFORT]$ | Pub, Sub | **1** | <span style="color:orange">Functional</span> |
| 18 | **PART ↔ PART** | $[Writer.PART \cap Reader.PART] = \emptyset$ | Pub ↔ Sub | **2** | <span style="color:red">Structural</span> |
| 19 | **RELIAB ↔ RELIAB** | $[Writer.RELIAB < Reader.RELIAB]$ | Pub ↔ Sub | **2** | <span style="color:red">Structural</span> |
| 20 | **DURABL ↔ DURABL** | $[Writer.DURABL < Reader.DURABL]$ | Pub ↔ Sub | **2** | <span style="color:red">Structural</span> |
| 21 | **DEADLN ↔ DEADLN** | $[Writer.DEADLN > Reader.DEADLN]$ | Pub ↔ Sub | **2** | <span style="color:red">Structural</span> |
| 22 | **LIVENS ↔ LIVENS** | $[W.LIVENS < R.LIVENS] \lor [W.lease > R.lease]$ | Pub ↔ Sub | **2** | <span style="color:red">Structural</span> |
| 23 | **OWNST ↔ OWNST** | $[Writer.OWNST \neq Reader.OWNST]$ | Pub ↔ Sub | **2** | <span style="color:red">Structural</span> |
| 24 | **DESTORD ↔ DESTORD** | $[Writer.DESTORD < Reader.DESTORD]$ | Pub ↔ Sub | **2** | <span style="color:red">Structural</span> |
| 25 | **WDLIFE → RDLIFE** | $[W.autodispose=FALSE] \land [R.autopurge\_disposed > 0]$ | Pub ↔ Sub | **2** | <span style="color:purple">Operational</span> |
| 26 | **WDLIFE → RDLIFE** | $[W.autodispose=FALSE] \land [R.autopurge\_nowriter = 0]$ | Pub ↔ Sub | **2** | <span style="color:orange">Functional</span> |
| 27 | **WDLIFE → RDLIFE** | $[W.autodispose=FALSE] \land [R.autopurge\_nowriter = \infty]$ | Pub ↔ Sub | **2** | <span style="color:purple">Operational</span> |
| 29 | **HIST → RELIAB** | $[RELIABLE] \land [KEEP\_LAST] \land [depth < \lceil RTT/PP \rceil + 2]$ | Pub | **3** | <span style="color:orange">Functional</span> |
| 30 | **RESLIM → RELIAB** | $[RELIABLE] \land [KEEP\_ALL] \land [mpi < \lceil RTT/PP \rceil + 1]$ | Pub | **3** | <span style="color:orange">Functional</span> |
| 31 | **LFSPAN → RELIAB** | $[RELIABLE] \land [LFSPAN.duration < RTT \times 2]$ | Pub | **3** | <span style="color:orange">Functional</span> |
| 33 | **RELIAB → DEADLN** | $[DEADLN.period > 0] \land [RELIAB=BEST\_EFFORT]$ | Sub | **3** | <span style="color:orange">Functional</span> |
| 34 | **LIVENS → DEADLN** | $[DEADLN.period > 0] \land [LIVENS.lease < DEADLN.period]$ | Sub | **3** | <span style="color:orange">Functional</span> |
| 36 | **DEADLN → OWNST** | $[OWNST=EXCLUSIVE] \land [DEADLN.period < 2 \times PP]$ | Sub | **3** | <span style="color:purple">Operational</span> |
| 37 | **LIVENS → OWNST** | $[OWNST=EXCLUSIVE] \land [lease < 2 \times PP]$ | Sub | **3** | <span style="color:purple">Operational</span> |
| 38 | **RELIAB → WDLIFE** | $[autodispose=TRUE] \land [RELIAB=BEST\_EFFORT]$ | Pub | **3** | <span style="color:orange">Functional</span> |
| 39 | **HIST → DURABL** | $[DURABL \ge TRAN\_LOCAL] \land [KEEP\_ALL] \land [mpi \ge default]$ | Pub | **3** | <span style="color:purple">Operational</span> |
| 40 | **DURABL → DEADLN** | $[DEADLN.period > 0] \land [DURABL \ge TRAN\_LOCAL]$ | Sub | **3** | <span style="color:purple">Operational</span> |


---  

## 🖥️ 실행 결과 예시

터미널에서 다음과 같은 색상 코드로 위반 사항이 보고됩니다:

<img src="qos_guard_example.png" width="600" height="400"/>

> 🔴 **빨간색 (Cat A)**: Structural Violation  🟠 **주황색 (Cat B)**: Functional Violation  🟣 **보라색 (Cat C)**: Operational Violation
>

---

## 📢 지원 정보 및 공지

### 지원 버전 현황

| ROS 2 버전 | Fast DDS | RTI Connext | Cyclone DDS |
| --- | --- | --- | --- |
| **humble** | 2.6.9 | 6.0.1 | 버전 무관 |
| **jazzy** | 2.14.0 | 6.0.1 | 버전 무관 |
| **kilted** | 2.14.4 | 7.3.0 | 버전 무관 |

- **Fast DDS**: 위 버전 조합에서 XML 프로파일과 소스 코드 병합 지원
- **Cyclone DDS**: 패키지 모드에서 **소스 코드 전용** 지원. XML QoS 미지원, Cyclone 버전 무관
- **RTI Connext**: XML 파서 미구현. 패키지 모드 시 소스 코드 스캔만 동작

### 향후 업데이트 계획
- GUI 업데이트

### 📢 제보 및 문의

QoS Guard는 지속적으로 진화하고 있습니다. 도구 사용 중 발견된 **버그(Error Report)**나, 실무에서 경험하신 **새로운 QoS 규칙(New QoS Rules)**이 있다면 언제든지 제보해 주세요. 여러분의 피드백이 로봇 통신의 안정성을 높입니다.

- **Email**: [leesh2913@dgist.ac.kr](mailto:leesh2913@dgist.ac.kr)
- **GitHub Issues**: [Project Issues Page]
- **Researcher Homepage**: [hun0130.github.io](https://hun0130.github.io/)

연구 협업 및 산학협력 파트너십도 환영합니다!

---

## 📄 라이선스

본 프로젝트는 라이선스 하에 배포됩니다. 자세한 내용은 LICENSE 파일을 참조하세요.


