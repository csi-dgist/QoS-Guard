# QoS Guard

**Catch broken ROS 2 QoS settings before they reach your robot.**

<p align="center">
  <img alt="ROS 2" src="https://img.shields.io/badge/ROS_2-Humble_%7C_Jazzy_%7C_Kilted-22314E?logo=ros&logoColor=white">
  <img alt="DDS" src="https://img.shields.io/badge/DDS-Fast_%7C_Cyclone_%7C_Connext-3fa037">
  <img alt="Python" src="https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white">
  <img alt="License" src="https://img.shields.io/badge/License-MIT-yellow">
</p>

QoS Guard is a static analyzer for ROS 2 Quality-of-Service (QoS) settings. It reads the QoS a package configures in XML profiles and source code, pairs each publisher with its subscriber by topic, and checks every pair against 40 dependency-violation rules grounded in the OMG DDS standard, the Fast DDS and Cyclone DDS implementations, and controlled measurements. It reports the risky configurations before deployment, with no live ROS 2 session, no robot, and no runtime required.

Full documentation and the rule reference: **https://csi-dgist.github.io/QoS-Guard/**

## Paper summary

ROS 2 relies on the Data Distribution Service (DDS), which offers more than 20 Quality-of-Service (QoS) policies governing availability, reliability, and resource utilization. Yet ROS 2 users lack clear guidance on safe policy combinations and validation processes prior to deployment, which often leads to trial-and-error tuning and unexpected runtime failures.

To address these challenges, we analyze DDS publisher-subscriber communication over a lifecycle divided into Discovery, Data Exchange, and Disassociation, and provide a user-oriented tutorial explaining how 16 QoS policies operate across these phases. We derive a QoS Policy Chain that encodes inter-policy relationships as 40 dependency-violation rules grounded in the OMG DDS standard, open-source implementations, and empirical measurements.

Finally, we introduce QoS Guard, a ROS 2 tool that statically extracts endpoint QoS settings from a package, validates them offline, flags conflicts, and enables pre-deployment risk detection without establishing a live ROS 2 session. Together, these contributions give ROS 2 users both conceptual insight and a concrete tool that enables early detection of misconfigurations, improving the reliability and resource efficiency of ROS 2-based robotic systems.

## Installation

### Option A: as a ROS 2 package (recommended if you use ROS 2)

**1. Clone into your workspace src**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/csi-dgist/QoS-Guard.git qos-guard
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

### Option B: standalone (no ROS 2)

You only need the repository and Python 3.10 or higher.

```bash
cd /path/to/QoS-Guard
python3 -m qos_guard.qos_checker /path/to/your_ros2_package
```

## Usage

### Check a package (default)

```bash
qos_guard <package_path> [dds] [ros_version]
qos_guard /path/to/package cyclone jazzy
```

`dds` is `fast`, `cyclone`, or `connext` (default `fast`). `ros_version` is `humble`, `jazzy`, or `kilted` (default `humble`).

### Set the publish period and RTT

The time-based rules depend on each topic's publish period (PP) and the deployment round-trip time (RTT). By default QoS Guard uses a conservative 100 ms PP and a 50 ms RTT. Pass `--auto-pp` to derive each topic's PP from the timer that drives it, or pin the values explicitly.

```bash
qos_guard /path/to/package cyclone jazzy --auto-pp
qos_guard /path/to/package cyclone jazzy publish_period=100ms rtt=50ms
```

### Verify a single writer/reader XML pair (Fast DDS, Connext)

```bash
qos_guard --xml <pub.xml> <sub.xml> fast humble
```

Cyclone DDS does not support XML QoS profiles, so use package mode for Cyclone.

### List the XML profiles a package exposes

```bash
qos_guard --list <package_path>
```

## Report

QoS Guard groups findings by severity.

- **Structural**: the endpoints are incompatible under the DDS Requested-Offered rules and will not connect.
- **Functional**: the endpoints connect, but a guarantee such as reliability or durability is silently broken.
- **Operational**: the configuration works but wastes memory or bandwidth.

When nothing is flagged, it prints `All Entities are safe !`.

## Requirements

| Requirement | Notes |
| --- | --- |
| Python | 3.10 or higher |
| ROS 2 | Optional (Humble, Jazzy, or Kilted) |
| DDS | Fast DDS, Cyclone DDS, or RTI Connext DDS |
| OS | Linux recommended. Windows works for Python-only use |

Tested with Fast DDS 2.6.11 and 2.14.6, Cyclone DDS 0.10.5, and RTI Connext DDS 6.0.1.

## Citation

If you use QoS Guard in your research, please cite:

> S. Lee, J. Kang, and K.-J. Park, "Dependency Chain Analysis of ROS 2 DDS QoS Policies: From Lifecycle Tutorial to Static Validation."

## License

Released under the [MIT License](LICENSE), Copyright (c) 2026 DGIST CSI LAB.
