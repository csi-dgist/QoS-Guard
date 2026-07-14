# QoS Guard: ROS 2 DDS QoS ROS 2 DDS QoS Policy Dependency Analysis and Static Validation Tool

<p align="center">
  <img alt="ROS2 logo" src="https://img.shields.io/badge/ROS--2-Humble%20%7C%20Jazzy%20%7C%20Kilted-blue?style=for-the-badge">
  <img alt="Fast DDS logo" src="https://img.shields.io/badge/Fast--DDS-2.6.9%20%7C%202.14.x-brightgreen?style=for-the-badge">
  <img alt="RTI Connext logo" src="https://img.shields.io/badge/RTI%20Connext-6.0.1%20%7C%207.3.0-orange?style=for-the-badge">
  <img alt="Cyclone DDS logo" src="https://img.shields.io/badge/Cyclone%20DDS-All%20Versions-lightgrey?style=for-the-badge">
  <img alt="Python logo" src="https://img.shields.io/badge/Python-3.10+-blue?style=for-the-badge">
</p>

## 📝 Project Overview

ROS 2 is built on top of the Data Distribution Service (DDS) and utilizes more than 20 Quality of Service (QoS) policies to control communication availability, reliability, and resource consumption. However, in real-world environments, developers often face a lack of clear guidance or pre-verification procedures for combining these policies, leading to trial-and-error-based tuning or unexpected runtime errors.

To address these challenges, this project breaks down DDS publisher-subscriber communication into three distinct stages: Discovery, Data Exchange, and Disassociation, providing a tutorial-style explanation of how 16 core QoS policies operate within each stage. Furthermore, by systematically analyzing the interdependencies between policies, we derived the QoS Dependency Chain and classified 40 common constraints into Dependency-Violation Rules.

Based on this comprehensive analysis, we developed the **QoS Guard package**. This tool allows users to **verify ROS 2 packages offline** prior to deployment, successfully detecting potential conflicts in advance. This enables safe and robust QoS configurations without the need to run an active ROS 2 session.

By providing both conceptual insights and practical tools, this work assists ROS 2 users in better understanding and managing QoS policies, ultimately enhancing the reliability of robotic communication and the efficiency of resource utilization.

## 💡 How to Run

This tool can be executed either as a **ROS 2 package** or as a **standalone Python script**. It does not require a running ROS 2 runtime and utilizes only standard Python libraries.

### Execution Modes

1. **Package Mode** (Default): Automatically scans and verifies internal QoS XML files by specifying the path to a ROS 2 package.  
2. **XML Pair Mode** (Optional): Directly specifies two files, `pub.xml` and `sub.xml`, for verification using the `--xml` option.
3. **List Mode** (Optional): Outputs a list of all XML files found within the package path using the `--list` option.

### Execution Arguments

- `package_path`: Path to the ROS 2 package (Package Mode, default)
- `--xml`: Activates XML Pair Mode, requires specifying `pub.xml` and `sub.xml` files
- `--list`: Activates List Mode
- `pub.xml` / `sub.xml`: Paths to the Writer/Reader QoS profiles (for XML Pair Mode)
- `dds`: DDS vendor – `fast` | `cyclone` | `connext`
- `ros_version`: ROS 2 version – `humble` | `jazzy` | `kilted`
- `publish_period`: Writer's message publishing period (PP), e.g., `40ms` (Optional, default: 40ms)
- `rtt`: Expected Round-Trip Time (RTT), e.g., `50ms` (Optional, default: 50ms)

## 💡 DDS Vendor Support Status

| **DDS Vendor** | **XML Profile Support** | **Source Code Scan** | **Remarks** |
| --- | --- | --- | --- |
| **Fast DDS** | ✓ | ✓ | Supports merged analysis of XML and source code QoS configurations |
| **RTI Connext** | **✓** | **✓** | **Full support for XML parser and profile matching** |
| **Cyclone DDS** | **N/A** | ✓ | **Does not support XML QoS by design** (Source code scan only) |

> ⚠️ **Cyclone DDS Notice**: Cyclone DDS does not provide a separate XML QoS profile format due to its middleware design specification. Therefore, when using Cyclone DDS, this tool performs its analysis by **directly parsing QoS configurations inside the source code** (such as `rclcpp::QoS`).
>

## 💡 Support Matrix by DDS Vendor and Version

This tool tracks the differing QoS interface support status for each DDS vendor and ROS 2 version, applying it directly to the verification process.

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

> **Legend:** O (Supported), X (Unsupported), △ (Partially/Conditionally Supported)
>

### External XML Profiles (Environment Variables)

This is a feature exclusive to Fast DDS. When running in Package Mode with `dds=fast`, the tool also scans **external XML files** specified by the following environment variables to perform a comprehensive analysis of the system-wide QoS configurations.

| Environment Variable | Description | Purpose |
| --- | --- | --- |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | Path to the Fast DDS default profile XML file | System default QoS settings |
| `RMW_FASTRTPS_CONFIG_FILE` | Path to the XML configuration file for ROS 2 rmw_fastrtps | ROS 2 middleware QoS settings |

Since external XML paths vary across user environments, **users must configure these environment variables manually**. If the environment variables are not set, the tool will only scan the XML files located inside the package.

```bash
# Example of setting environment variables before execution
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/default_profiles.xml
qos_guard /path/to/package fast humble

# Alternatively, apply it temporarily with the command
FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/default_profiles.xml qos_guard /path/to/package fast humble
```

---

## 🔧 Installation and Setup

### Method A: Install and Run as a ROS 2 Package

```bash
# 1. Create a ROS 2 workspace (if you don't have one already)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone the repository
git clone --branch QosGuard_v3 https://github.com/QosGuard-Anonymous/qos-guard.github.io.git

# 3. Build the package
cd ~/ros2_ws
colcon build --packages-select qos_guard
source install/setup.bash

# 4. Execution Examples
ros2 run qos_guard qos_guard /path/to/ros2_package                    # Package Mode (Default)
ros2 run qos_guard qos_guard /path/to/ros2_package fast jazzy        # Package Mode (Specific DDS/ROS)
ros2 run qos_guard qos_guard --xml pub.xml sub.xml fast humble         # XML Pair Mode
ros2 run qos_guard qos_guard --list /path/to/package                 # Print list of XML files
```

### Method B: Run as a Standalone Python Script

```bash
# 1. Clone the repository
git clone --branch QosGuard_v3 https://github.com/QosGuard-Anonymous/qos-guard.github.io.git
cd qos-guard.github.io/qos_guard

# 2. Run directly using Python (Requires Python 3.10+)
python3 -m qos_guard.qos_checker /path/to/ros2_package                    # Package Mode (Default)
python3 -m qos_guard.qos_checker /path/to/ros2_package fast jazzy        # Package Mode (Specific DDS/ROS)
python3 -m qos_guard.qos_checker --xml pub.xml sub.xml fast humble         # XML Pair Mode
python3 -m qos_guard.qos_checker --list /path/to/package                 # Print list of XML files
```

**Important**: You do not need ROS 2 installed to use this method; only Python 3.10+ is required.

---

## 📂 Project Structure

```
qos_guard/
├── qos_guard/                    # Main Python package
│   ├── __init__.py               # Package initialization
│   ├── cli.py                    # CLI argument parsing and command handling
│   ├── xml_parser.py              # XML QoS profile parsing module
│   ├── rules_fastdds_humble.py   # Rule checking engine for Fast DDS + Humble
│   ├── package_scanner.py        # Automatic ROS 2 package QoS XML scanner
│   ├── output.py                 # Verification results output formatter
│   └── qos_checker.py            # Main entry point and orchestration logic
├── resource/                     # Resource directory
│   └── qos_guard                 # Additional resource tracking
├── test/                         # Unit tests directory
│   ├── test_copyright.py          # Copyright verification test
│   ├── test_flake8.py            # Code style test
│   └── test_pep257.py            # Documentation style test
├── test_xml/                     # XML profiles for testing
│   ├── pub.xml                   # Sample Writer QoS profile
│   └── sub.xml                   # Sample Reader QoS profile
├── package.xml                   # ROS 2 package metadata
├── setup.cfg                     # Package configuration metadata
└── setup.py                      # Python package installation script
```

---

## 🧪 Key Features

This tool parses and validates the following QoS configurations:

### Supported QoS Policies
- `ENTITY_FACTORY`, `PARTITION`, `USER_DATA`, `GROUP_DATA`, `TOPIC_DATA`
- `RELIABILITY`, `DURABILITY`, `DEADLINE`, `LIVELINESS`
- `HISTORY`, `RESOURCE_LIMITS`, `LIFESPAN`
- `OWNERSHIP(+STRENGTH)`, `DESTINATION_ORDER`
- `WRITER_DATA_LIFECYCLE`, `READER_DATA_LIFECYCLE`


## 📝 QoS-Guard: Fast DDS Profile Priority Guide (Humble ~ Kilted)

This project strictly adheres to the following precedence rules to prevent QoS configuration conflicts when using Fast DDS (rmw_fastrtps) across ROS 2 Humble, Jazzy, and Kilted.

### 🔝 QoS Application Hierarchy

When QoS settings are defined in multiple locations, **the highest priority setting completely overrides any conflicting lower-level configurations.**

| Priority | Configuration Location | Matching Method | Remarks |
| --- | --- | --- | --- |
| **1 ((Highest)** | **ROS 2 Source Code (`rclcpp`)** | `rclcpp::QoS` (non-DEFAULT) | Completely ignores XML configurations |
| **2** | **XML: `<topic profile_name="...">`** | Automatic Topic Name Matching | [Highly Recommended] Universal XML layer |
| **3** | **XML: `<data_writer>` / `<data_reader>`** | Automatic Topic Name Matching | Jazzy/Kilted native style |
| **4** | **XML: `<publisher>` / `<subscriber>`** | Explicit Name Matching in Code | Humble style (includes inline <topic>) |
| **5 (Lowest)** | **XML: `is_default_profile="true"`** | Fallback (Default) | Applied only when no explicit settings exist |

---

### 💡 Version Characteristics and Matching Behavior

* **Best Practice:** To ensure an overriding configuration regardless of the ROS 2 version, use the Priority 2 (<topic>) tag.
* **Humble:** Primarily uses the Priority 4 <publisher profile_name="my_pub"> approach, which requires explicit matching via PublisherOptions in the source code.
* **Jazzy/Kilted:** Introduced the Priority 3 <data_writer profile_name="/topic_name"> convention, allowing automatic matching based strictly on the topic name.
---

### 🔍 Verifying the Effectively Applied QoS

To verify which QoS settings are actively applied to a specific topic at runtime, use the following command:

```bash
# Check the detailed active QoS profile of a specific topic
ros2 topic echo /your_topic_name --qos-profile all
```

---

## 📎 Package Mode: QoS Profile Matching Rules

This section defines how publisher and subscriber XML files are paired together during Package Mode execution (qos_guard /path/to/pkg fast humble).

### 1. Base Name Extraction (Regex)

The system extracts a pure base name from the profile_name by stripping out suffixes such as _pub, _subscriber, _writer, _reader, and _profile.

| profile_name | Extracted base |
| --- | --- |
| `cmd_vel_pub` | `cmd_vel` |
| `cmd_vel_subscriber` | `cmd_vel` |
| `latency_publisher_profile` | `latency` |
| `datawriter_profile_example` | `datawriter_profile_example` |

→ cmd_vel_pub and cmd_vel_subscriber are recognized as having the same base and will be paired together.

### 2. Matching Rules Detail

| Condition | Operational Behavior |
| --- | --- |
| **Identical base** | Forms an explicit pair containing only that specific pub ↔ sub for verification. |
| **No profile_name** | Evaluates all possible pub × sub combinations for full verification. |
| **Contains Wildcard Keyword** | If default, common, or generic is detected → Pairs with all available pub × sub combinations. |

### 3. Wildcard Keyword Expansion

If keywords like the following are found in the base string (e.g., generic_qos_pub, default_profile_sub, common_publisher), the profile will be cross-checked against all other profiles:

- `default`
- `common` 
- `generic`

This ensures that general-purpose profiles are evaluated globally across multiple topics.

### 4. Matching Algorithm Summary

```
1. Extract base from profile_name (removes _pub, _sub, etc., via regex).
2. Does the extracted base contain default/common/generic? → Generate all combinations (Wildcard).
3. Otherwise: Pair only pub-sub entities that share the exact same base name.
4. Missing profile_name? → Fall back to generating all combinations.
```

---


## 🧪 Validation Classification System

The tool scans for over 40 dependency rule violations between Writer and Reader profiles, classifying them based on their overall system impact:

- **<span style="color:red">Category A: Structural Violation (Structural Violation)</span>**
    - Violations cause matching failures at the RMW level or induce abnormal system terminations (e.g., Segfaults).
- **<span style="color:orange">Category B: Functional Violation (Functional Violation)</span>**
    - Communication is established, but the intended quality of service contracts (such as Reliable data delivery or Persistence guarantees) are broken.
- **<span style="color:purple">Category C: Operational Violation (Operational Violation)</span>**
    - No functional failures occur, but it leads to resource inefficiencies like unnecessary memory consumption or bandwidth waste.

## 📋 QoS Guard Rules (Ordered by Stage)

These are the 40 core rules utilized by QoS Guard during offline validation.

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

## 📢 Support Information and Announcements

### Version Support Matrix

| ROS 2 Version | Fast DDS | RTI Connext | Cyclone DDS |
| --- | --- | --- | --- |
| **humble** | 2.6.9 | 6.0.1 | Version Agnostic |
| **jazzy** | 2.14.0 | 6.0.1 | Version Agnostic |
| **kilted** | 2.14.4 | 7.3.0 | Version Agnostic |

- **Fast DDS**: Supports merged analysis of XML profiles and source code under the specified version combinations.
- **Cyclone DDS**: Supports source code analysis only in Package Mode. Does not support XML QoS; compatible with all Cyclone versions.
- **RTI Connext**: XML parser is not yet implemented. Package Mode falls back to scanning source code only.
  
### Future Roadmap
- GUI update

### 📢 Bug Reporting & Inquiries

QoS Guard is continuously evolving. If you encounter any bugs (Error Reports) while using the tool, or if you would like to propose new QoS rules based on your field experience, please feel free to submit them. Your feedback directly impacts the reliability of robotic communication systems.

- **Email**: [leesh2913@dgist.ac.kr](mailto:leesh2913@dgist.ac.kr)
- **GitHub Issues**: [Project Issues Page]
- **Researcher Homepage**: [hun0130.github.io](https://hun0130.github.io/)

We warmly welcome research collaborations and industry-academic partnerships!

---

## 📄 License

This project is distributed under the license terms. For detailed information, please refer to the LICENSE file.


