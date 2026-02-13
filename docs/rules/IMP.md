# Stage 2: RxO (Required-versus-Offered) Rules

서로 다른 노드 간의 QoS 프로파일 호환성을 검사합니다. (총 8개 규칙)

| 파라미터 | 호환성 규칙 (Offered vs Requested) |
| :--- | :--- |
| **Reliability** | Offered Reliable $\ge$ Requested Best Effort |
| **Durability** | Offered Transient Local $\ge$ Requested Volatile |
| **Deadline** | Offered Deadline $\le$ Requested Deadline |
| **Liveliness** | Offered Automatic $\ge$ Requested ManualByTopic |

!!! warning "주의"
    이 규칙이 위반될 경우 ROS 2 노드 간 통신이 이루어지지 않습니다.
