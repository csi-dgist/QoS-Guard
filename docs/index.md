# QoS Guard: ROS 2 QoS Static Verifier

**Dependency Chain Analysis of ROS 2 DDS QoS Policies: From Lifecycle Tutorial to Static Verification**

---

## 📝 Abstract

ROS 2 is built on the **Data Distribution Service (DDS)** and leverages more than 20 **Quality of Service (QoS)** policies to control communication availability, reliability, and resource usage. 

However, users often lack clear guidance, leading to:
* ⚠️ **Trial-and-error tuning**
* ⚠️ **Unexpected runtime failures**

To address this, we introduce **QoS Guard**, a tool for **offline verification** of DDS XML profiles to detect potential conflicts **before deployment**.

---

## 🚀 Key Contributions

!!! abstract "Three-Phase Decomposition"
    We decompose DDS communication into **Discovery, Data Exchange, and Disassociation**, providing tutorial-style insights for 16 key QoS policies.

!!! success "40+ Dependency-Violation Rules"
    Systematic analysis of inter-policy dependencies, deriving a **QoS Dependency Chain** and 40 common constraints.

!!! gear "Practical Tool: QoS Guard"
    Enables users to safely configure QoS settings without launching a ROS 2 session, improving communication reliability and resource efficiency.

---

## 🛠️ Quick Start

### 1. Installation
```bash
# Clone the repository
git clone --branch QosGuard_v3 [https://github.com/csi-dgist/QoS-Guard.git](https://github.com/csi-dgist/QoS-Guard.git)

# Build the package
colcon build --packages-select check_qos
