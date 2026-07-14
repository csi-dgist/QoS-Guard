# qos_test_pkg

A test package designed for validating the QoS-Guard static analysis tool.  
It is specifically designed to confuse and validate **static analyzers (XML parsers, code analyzers)** rather than for actual runtime execution.

## Topic Structure (Topology)

| Topic | Type | Structure | Description |
|------|------|------|------|
| `/cmd_vel` | Topic A | 1:1 | Simple Pub-Sub |
| `/sensor_data` | Topic B | 1:N | 1 Pub : 3 Subs (Each Sub has a different QoS) |
| `/map` | Topic C | N:1 | 2 Pubs : 1 Sub |
| `/status` | Topic D | Isolated | No Pub/Sub connection targets |

## QoS Configuration Patterns (5 Types)

| Pattern | Target Location | Remarks |
|------|-----------|------|
| **Code Only** | `cmd_vel_pub` | Configures QoS only within `rclcpp` without XML. |
| **XML Topic Profile** | `/sensor_data`, `/map`, `/status` | `<topic profile_name="...">` takes the highest priority. |
| **XML Entity Profile** | `cmd_vel_sub`, `map_sub`, `status_sub` | Humble: `<publisher>`/`<subscriber>`, Jazzy: `<data_writer>`/`<data_reader>` |
| **Mixed Priority** | `/status` | Conflicts between Code (Reliable), XML Entity (BestEffort), and Topic Profile (KEEP_ALL). |
| **Default Trap** | `status_pub` | Enforces the `is_default_profile="true"` profile. |

## profiles/ XML Files

| File | Style | Description / Content |
|------|--------|------|
| `entity_profiles_humble.xml` | Humble | `<subscriber>` profiles (`cmd_vel`, `map`, `status`) |
| `topic_profiles.xml` | - | `<topic profile_name="/topic_name">` topic profiles |
| `profiles_jazzy.xml` | Jazzy | `<data_writer>` and `<data_reader>` profiles |
| `default_trap.xml` | - | `is_default_profile="true"` (`participant`, `publisher`, `subscriber`) |

## Build and Execution

```bash
cd /path/to/workspace
colcon build --packages-select qos_test_pkg
source install/setup.bash

# Launch all nodes
ros2 launch qos_test_pkg test_all.launch.py
```

## Verification with QoS-Guard
```bash
# Static analysis at the package path
qos_guard scan /path/to/qos_test_pkg
```

