# Dependency Chain Analysis of ROS 2 DDS QoS Policies: From Lifecycle Tutorial to Static Verification
<p align="center">
  <img alt="ROS2 logo" src="https://img.shields.io/badge/ROS--2-Humble-blue?style=for-the-badge">
  <img alt="Fast DDS logo" src="https://img.shields.io/badge/Fast--DDS-2.6.9-brightgreen?style=for-the-badge">
</p>

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

/* 표 형태 섹션: 첫 번째 열 너비 통일 */
.req-container.req-table .req-label {
    width: 220px;
    min-width: 220px;
    max-width: 220px;
    flex-shrink: 0;
    box-sizing: border-box;
}

.req-value {
    color: #334155; /* 기준 색상 */
    font-size: 14px;
    border-left: 2px solid #e2e8f0;
    padding-left: 16px;
    margin-left: 8px;
}

/* DDS support: 헤더 + 3열 표 (열 비율 1.5:1:1, 가운데 정렬) */
.dds-table .dds-header {
    display: grid;
    grid-template-columns: 1.5fr 1fr 1fr;
    gap: 16px;
    padding: 12px 16px;
    background: linear-gradient(180deg, #f8fafc 0%, #f1f5f9 100%);
    border-bottom: 2px solid #e2e8f0;
    font-weight: 700;
    font-size: 12px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: #475569;
    text-align: center;
}
.dds-table .dds-header span {
    text-align: center;
}
.dds-table .dds-row {
    display: grid;
    grid-template-columns: 1.5fr 1fr 1fr;
    gap: 16px;
    align-items: center;
    padding: 12px 16px;
    border-bottom: 1px solid #e2e8f0;
    font-size: 14px;
    color: #334155;
}
.dds-table .dds-row:last-child {
    border-bottom: none;
}
.dds-table .dds-row .dds-name {
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: #334155;
    text-align: center;
}
.dds-table .dds-cell {
    text-align: center;
}

/* Verification results: severity cards */
.severity-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 16px;
    margin: 20px 0;
}
.severity-card {
    padding: 18px;
    background: #ffffff;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    border-left: 4px solid #94a3b8;
    transition: all 0.2s ease;
}
.severity-card:hover {
    box-shadow: 0 4px 12px rgba(0,0,0,0.06);
}
.severity-card.structural {
    border-left-color: #dc2626;
    background: linear-gradient(135deg, #fef2f2 0%, #fff 100%);
}
.severity-card.functional {
    border-left-color: #ea580c;
    background: linear-gradient(135deg, #fff7ed 0%, #fff 100%);
}
.severity-card.operational {
    border-left-color: #4E5EB4;
    background: linear-gradient(135deg, #f8fafc 0%, #fff 100%);
}
.severity-card .severity-title {
    font-weight: 700;
    font-size: 14px;
    color: #1e293b;
    margin-bottom: 8px;
    text-transform: uppercase;
    letter-spacing: 0.04em;
}
.severity-card .severity-desc {
    font-size: 13px;
    color: #64748b;
    line-height: 1.5;
    margin-bottom: 10px;
}
.severity-card .severity-action {
    font-size: 12px;
    color: #475569;
    padding-top: 10px;
    border-top: 1px solid #f1f5f9;
    font-style: italic;
}
@media (max-width: 768px) {
    .severity-grid {
        grid-template-columns: 1fr;
    }
}

/* FAQ: Q&A 카드 리스트 */
.faq-list {
    margin: 20px 0;
    display: flex;
    flex-direction: column;
    gap: 12px;
}
.faq-item {
    padding: 16px 18px;
    background: #ffffff;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    border-left: 4px solid #4E5EB4;
    transition: all 0.2s ease;
}
.faq-item:hover {
    border-color: #c7d2fe;
    box-shadow: 0 2px 8px rgba(78, 94, 180, 0.06);
}
.faq-item .faq-q {
    font-weight: 700;
    font-size: 14px;
    color: #1e293b;
    margin-bottom: 8px;
    line-height: 1.4;
}
.faq-item .faq-a {
    font-size: 13px;
    color: #64748b;
    line-height: 1.55;
    padding-left: 0;
}
.faq-item .faq-a code {
    background: #f1f5f9;
    padding: 2px 6px;
    border-radius: 4px;
    font-size: 12px;
}
</style>


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


