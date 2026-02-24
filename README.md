# Dependency Chain Analysis of ROS 2 DDS QoS Policies: From Lifecycle Tutorial to Static Validation
<p align="center">
  <img alt="ROS2 logo" src="https://img.shields.io/badge/ROS--2-Humble-blue?style=for-the-badge">
  <img alt="Fast DDS logo" src="https://img.shields.io/badge/Fast--DDS-2.6.9-brightgreen?style=for-the-badge">
</p>


## 📝 Paper Summary
ROS 2 is built on the Data Distribution Service (DDS) and leverages more than 20 Quality of Service (QoS) policies to control communication availability, reliability, and resource usage. However, in practice, users often lack clear guidance or pre-verification procedures for combining these policies, which frequently forces them into trial-and-error tuning or results in unexpected runtime failures.
To address this challenge, we decompose DDS publisher–subscriber communication into three phases—Discovery, Data Exchange, and Disassociation—and provide a tutorial-style explanation of how 16 key QoS policies operate at each stage. We also systematically analyze inter-policy dependencies, deriving a QoS Dependency Chain, and classify 40 common constraints into a set of Dependency-Violation Rules.
Building on this analysis, we developed the QoS Guard package, which enables offline verification of DDS XML profiles to detect potential conflicts before deployment. This allows users to safely configure QoS settings without needing to launch a ROS 2 session.
By offering both conceptual insights and a practical tool, this work helps ROS 2 users better understand and manage QoS policies, ultimately improving the reliability of robot communications and the efficiency of resource utilization.

## 💡 How to run it from the terminal

### Quick Start

> **After installing as ROS 2 package (see Installation)**

```bash
qos_guard /path/to/your_ros2_package
```
> **Specify DDS and ROS version**

```bash
qos_guard /path/to/package fast jazzy
```
> **Without ROS 2: run from repo root**

```bash
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


**1. Clone into your workspace src**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository_URL> qos-guard
```

**2. Build and source**
```bash
cd ~/ros2_ws
colcon build --packages-select qos_guard
source install/setup.bash
```

**3. Run**
```bash
ros2 run qos_guard qos_guard /path/to/any/ros2/package
```

<hr class="hr-dashed">

### Option B: Standalone (no ROS 2)

You only need the repository and Python 3.10+.


**1. Clone and go to repo root**
```bash
cd /path/to/QoS-Guard
```

**2. Run**
```bash
python3 -m qos_guard.qos_checker /path/to/your_ros2_package
```

<hr class="hr-grad-left">


