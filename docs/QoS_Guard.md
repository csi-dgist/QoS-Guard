# QoS-Guard

<style>
  /* 특징 그리드 레이아웃 */
.feature-grid {
    display: grid;
    grid-template-columns: 1fr 1fr; /* 2열 배치 */
    gap: 16px;
    margin: 24px 0;
}

.feature-card {
    display: flex;
    align-items: flex-start;
    padding: 18px;
    background: #ffffff;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    transition: all 0.2s ease;
}

.feature-card:hover {
    border-color: #4E5EB4;
    box-shadow: 0 4px 12px rgba(78, 94, 180, 0.08);
    transform: translateY(-2px);
}

.feature-icon {
    font-size: 20px;
    margin-right: 14px;
    margin-top: 2px;
}

.feature-content strong {
    display: block;
    font-size: 15px;
    color: #1e293b;
    margin-bottom: 4px;
}

.feature-content span {
    font-size: 13px;
    color: #64748b;
    line-height: 1.5;
}

/* 모바일대응: 화면이 작아지면 1열로 변환 */
@media (max-width: 768px) {
    .feature-grid {
        grid-template-columns: 1fr;
    }
}

.req-container {
    margin: 20px 0;
    border: 1px solid #e2e8f0;
    border-radius: 8px;
    background-color: #ffffff; /* 흰색 배경 고정 */
    overflow: hidden;
}

.req-item {
    display: flex;
    align-items: center;
    padding: 12px 16px;
    border-bottom: 1px solid #e2e8f0;
    /* 호버 효과 없음 */
}

.req-item:last-child {
    border-bottom: none;
}

.req-label {
    min-width: 100px;
    font-weight: 700;
    color: #334155; /* 오른쪽 텍스트와 동일한 색상으로 수정 */
    font-size: 13px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
}

.req-value {
    color: #334155; /* 기준 색상 */
    font-size: 14px;
    border-left: 2px solid #e2e8f0;
    padding-left: 16px;
    margin-left: 8px;
}
</style>

**Offline static verification of DDS QoS for ROS 2.** 

Find QoS mismatches and dependency violations before you run your nodes—no ROS 2 runtime required.

<hr class="hr-grad-left">

## Overview

ROS 2 uses **DDS** and **QoS policies** (reliability, durability, history, etc.) for topic communication. 

If Publisher and Subscriber QoS do not match or conflict, you can get **failed connections**, **data loss**, or **crashes**.

QoS-Guard scans your **XML profiles** and **source code** (e.g. `rclcpp::QoS`, `create_publisher`), builds Pub–Sub pairs per topic, and runs **40 dependency rules** to report potential issues.

<div class="feature-grid">
  <div class="feature-card">
    <div class="feature-icon">⚡</div>
    <div class="feature-content">
      <strong>No runtime</strong>
      <span>No need to run <code>ros2 run</code> or any node</span>
    </div>
  </div>
  <div class="feature-card">
    <div class="feature-icon">💻</div>
    <div class="feature-content">
      <strong>Python 3.10+</strong>
      <span>Works with or without ROS 2 installed</span>
    </div>
  </div>
  <div class="feature-card">
    <div class="feature-icon">📂</div>
    <div class="feature-content">
      <strong>Package mode</strong>
      <span>Point to a package path <br>→ auto-scan XML + code, verify all topic pairs</span>
    </div>
  </div>
  <div class="feature-card">
    <div class="feature-icon">📄</div>
    <div class="feature-content">
      <strong>XML pair mode</strong>
      <span>Point to one pub XML + one sub XML <br>→ verify that pair only (Fast/Connext)</span>
    </div>
  </div>
</div>

<hr class="hr-grad-left">

## Quick Start

```bash
# After installing as ROS 2 package (see Installation)
qos_guard /path/to/your_ros2_package

# Specify DDS and ROS version
qos_guard /path/to/package fast jazzy

# Without ROS 2: run from repo root
cd /path/to/QoS-Guard
python3 -m qos_guard.qos_checker /path/to/package
```

<hr class="hr-grad-left">

## Requirements

<div class="req-container">
  <div class="req-item">
    <span class="req-label">Python</span>
    <span class="req-value">3.10 or higher</span>
  </div>
  <div class="req-item">
    <span class="req-label">ROS 2</span>
    <span class="req-value">Optional (Humble, Jazzy, or Kilted)</span>
  </div>
  <div class="req-item">
    <span class="req-label">OS</span>
    <span class="req-value">Linux recommended / Windows (Python only)</span>
  </div>
</div>

<hr class="hr-grad-left">

## Installation

### Option A: As a ROS 2 package (recommended if you use ROS 2)

After this, you run the tool with `ros2 run qos_guard qos_guard ...`.

```bash
# 1. Clone into your workspace src
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository_URL> qos-guard

# 2. Build and source
cd ~/ros2_ws
colcon build --packages-select qos_guard
source install/setup.bash

# 3. Verify
ros2 run qos_guard qos_guard /path/to/any/ros2/package
```

### Option B: Standalone (no ROS 2)

You only need the repository and Python 3.10+.

```bash
# 1. Clone and go to repo root (parent of the qos_guard folder)
cd /path/to/QoS-Guard

# 2. Optional: use a virtual environment
python3 -m venv .venv
source .venv/bin/activate   # Linux/macOS
# .venv\Scripts\activate    # Windows

# 3. Run (parent of qos_guard must be on Python path)
python3 -m qos_guard.qos_checker /path/to/your_ros2_package
```

If you run from inside the `qos_guard` package directory, run from the **parent** of that directory so the module is found, e.g.:

```bash
cd /path/to/QoS-Guard
python3 -m qos_guard.qos_checker ~/ros2_ws/src/my_pkg
```

---

## Usage

The tool has three modes. Examples below use the `qos_guard` command; if you use standalone Python, replace it with `python3 -m qos_guard.qos_checker` and keep the same arguments.

### 1. Package mode (default)

**When to use:** Check a whole ROS 2 package. The tool finds all `*.xml` (except `package.xml`) and scans `.cpp`, `.hpp`, `.h`, `.py` for publishers/subscribers, builds Pub–Sub pairs by topic, and runs the rule checks.

```bash
qos_guard <package_path> [dds] [ros_version] [publish_period=<N>ms] [rtt=<N>ms]
```

**Examples:**

```bash
# Defaults: DDS=fast, ROS=humble, publish_period=40ms, rtt=50ms
qos_guard ~/ros2_ws/src/my_robot_pkg

# Fast DDS + Jazzy
qos_guard ~/ros2_ws/src/my_robot_pkg fast jazzy

# Custom period and RTT (used by some rules, e.g. reliable + history depth)
qos_guard ~/ros2_ws/src/my_robot_pkg fast humble publish_period=20ms rtt=30ms
```

### 2. XML pair mode

**When to use:** You have one Writer QoS XML and one Reader QoS XML and want to verify only that pair. Supported for **Fast DDS** and **Connext** only.

```bash
qos_guard --xml <pub.xml> <sub.xml> <dds> <ros_version> [publish_period=<N>ms] [rtt=<N>ms]
```

**Examples:**

```bash
qos_guard --xml ./profiles/writer.xml ./profiles/reader.xml fast humble
qos_guard -x pub_qos.xml sub_qos.xml connext jazzy publish_period=10ms
```

> **Cyclone DDS** does not support XML QoS profiles. For Cyclone, use **package mode** only (code scan). The `--xml` option is not available when `dds=cyclone`.

### 3. List mode

**When to use:** See which XML files the tool would scan under a package (useful to confirm config layout).

```bash
qos_guard --list <package_path>
```

**Example output:**

```
Found 4 XML file(s) in /path/to/my_pkg
  /path/to/my_pkg/config/fastdds.xml
  /path/to/my_pkg/profiles/topic_profiles.xml
  ...
```

### Arguments summary

| Argument | Meaning | Default |
|----------|---------|---------|
| `package_path` | Path to the ROS 2 package directory | Required in package/list mode |
| `dds` | `fast`, `cyclone`, or `connext` | `fast` |
| `ros_version` | `humble`, `jazzy`, or `kilted` | `humble` |
| `publish_period=<N>ms` | Writer publish period (ms); used by rules that depend on period | `40ms` |
| `rtt=<N>ms` | Expected round-trip time (ms); used by reliability/depth rules | `50ms` |

---

## DDS support

| DDS | XML profiles | Source code scan | Notes |
|-----|--------------|------------------|--------|
| **Fast DDS** | ✓ | ✓ | XML and code are merged; 5-level priority applied. |
| **RTI Connext** | ✓ | ✓ | Full XML parsing and profile matching. |
| **Cyclone DDS** | — | ✓ | No XML QoS; use package mode and code scan only. |

---

## How QoS is applied (package mode)

When both XML and code define QoS, the **highest priority** source wins (and overrides lower ones):

| Priority | Source | Description |
|----------|--------|-------------|
| **L1** | Code (`rclcpp::QoS`, etc.) | Non-default values in code override XML. |
| **L2** | `<topic profile_name="/topic_name">` | Matched by topic name; works across ROS 2 versions. |
| **L3** | `<data_writer>` / `<data_reader>` | Jazzy/Kilted style; topic name matching. |
| **L4** | `<publisher>` / `<subscriber>` | Humble style; profile name from code. |
| **L5** | `is_default_profile="true"` | Fallback when nothing else matches. |

Pub–Sub **pairing** in package mode: same **topic name** → same pair. If there is no topic name, the tool derives a base name from `profile_name` (e.g. `cmd_vel_pub` and `cmd_vel_subscriber` → base `cmd_vel`) and pairs by that.

---

## Verification results

The tool reports violations by **severity**. Output is **color-coded** in the terminal; a **summary table** at the end shows counts per category.

| Severity | Meaning | What to do |
|----------|---------|------------|
| **Structural** | RMW-level incompatibility; connection can fail or process may crash. | Fix first; these block communication. |
| **Functional** | Connection may work but guarantees (reliability, durability, etc.) are broken. | Risk of data loss or late-joiner issues; fix when possible. |
| **Operational** | No functional bug but inefficient (e.g. extra memory or bandwidth). | Optional to fix; improves resource use. |

If there are **no violations**, you see: **`All Entities are safe !`**

---

## Test package

The repo includes **qos_test_pkg** to try the tool: multiple topics, mixed QoS styles (code-only, XML topic profile, entity profile, default trap).

```bash
cd ~/ros2_ws
colcon build --packages-select qos_test_pkg
source install/setup.bash

qos_guard ~/ros2_ws/src/qos_test_pkg fast humble
```

For topic layout and profile descriptions, see **`qos_test_pkg/README.md`**.

---

## Fast DDS: external XML (package mode)

When `dds=fast`, package mode can also load XML from these **environment variables** (so system-wide or workspace QoS is included):

| Variable | Purpose |
|----------|---------|
| `FASTRTPS_DEFAULT_PROFILES_FILE` | Fast DDS default profiles XML path. |
| `RMW_FASTRTPS_CONFIG_FILE` | ROS 2 rmw_fastrtps config XML path. |

**Example:**

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/default_profiles.xml
qos_guard /path/to/package fast humble
```

Or one-off:

```bash
FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/default_profiles.xml qos_guard /path/to/package fast humble
```

---

## FAQ

| Question | Answer |
|----------|--------|
| **Can I use it without ROS 2?** | Yes. Use `python3 -m qos_guard.qos_checker <package_path> [dds] [ros_version]` from the repo root. |
| **Why does `--xml` not work with Cyclone?** | Cyclone DDS has no XML QoS profile support; only package mode (code scan) is available. |
| **How do I check only one topic?** | Use XML pair mode with that topic’s pub/sub XML files, or a minimal package containing only that topic’s config/code. |
| **How do I see actual QoS at runtime?** | Use `ros2 topic echo /topic_name --qos-profile all`. QoS-Guard is for **static** checks before deployment. |
| **Exit code is 0 but violations were printed.** | The tool may still exit 0 when violations exist; use the printed report and summary table to see if anything failed. |

---

## More information

- **License:** See the **LICENSE** file in the project.
- **Full rule list, DDS/ROS matrices, detailed design:** See **Project.md**.
