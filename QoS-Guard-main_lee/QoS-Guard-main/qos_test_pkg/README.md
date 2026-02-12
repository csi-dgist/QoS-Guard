# qos_test_pkg

QoS-Guard 정적 분석 도구 검증용 테스트 패키지.  
실제 구동보다는 **정적 분석기(XML 파서, 코드 분석기)**를 혼란시키고 검증하는 목적으로 설계되었습니다.

## 토픽 구조 (Topology)

| 토픽 | 유형 | 구조 | 설명 |
|------|------|------|------|
| `/cmd_vel` | Topic A | 1:1 | 단순 Pub-Sub |
| `/sensor_data` | Topic B | 1:N | 1 Pub : 3 Sub (각 Sub QoS 상이) |
| `/map` | Topic C | N:1 | 2 Pub : 1 Sub |
| `/status` | Topic D | 고립 | Pub/Sub 연결 대상 없음 |

## QoS 설정 패턴 (5가지)

| 패턴 | 적용 위치 | 비고 |
|------|-----------|------|
| **Code Only** | `cmd_vel_pub` | XML 없이 rclcpp에서만 QoS 설정 |
| **XML Topic Profile** | `/sensor_data`, `/map`, `/status` | `<topic profile_name="...">` 최우선 |
| **XML Entity Profile** | `cmd_vel_sub`, `map_sub`, `status_sub` | Humble: `<publisher>`/`<subscriber>`, Jazzy: `<data_writer>`/`<data_reader>` |
| **Mixed Priority** | `/status` | 코드(Reliable), XML 엔티티(BestEffort), topic 프로파일(KEEP_ALL) 충돌 |
| **Default Trap** | `status_pub` | `is_default_profile="true"` 프로파일 강제 적용 |

## profiles/ XML 파일

| 파일 | 스타일 | 내용 |
|------|--------|------|
| `entity_profiles_humble.xml` | Humble | `<subscriber>` 프로파일 (cmd_vel, map, status) |
| `topic_profiles.xml` | - | `<topic profile_name="/토픽명">` 토픽 프로파일 |
| `profiles_jazzy.xml` | Jazzy | `<data_writer>`, `<data_reader>` 프로파일 |
| `default_trap.xml` | - | `is_default_profile="true"` (participant, publisher, subscriber) |

## 빌드 및 실행

```bash
cd /path/to/workspace
colcon build --packages-select qos_test_pkg
source install/setup.bash

# 전체 노드 런치
ros2 launch qos_test_pkg test_all.launch.py
```

## QoS-Guard로 검증

```bash
# 패키지 경로에서 정적 분석
qos_guard scan /path/to/qos_test_pkg
```
