<h1 align="center">QoS Guard: ROS 2 QoS Static Validator</h1>

<p align="center">
  <strong>Dependency Chain Analysis of ROS 2 DDS QoS Policies: From Lifecycle Tutorial to Static Validation</strong>
</p>
<p align="center">
  <img alt="ROS2 logo" src="https://img.shields.io/badge/ROS--2-Humble-blue?style=for-the-badge">
  <img alt="Fast DDS logo" src="https://img.shields.io/badge/Fast--DDS-2.6.9-brightgreen?style=for-the-badge">
</p>

<h2 align="center">Abstract</h2>

<hr class="hr-grad-center">

ROS 2 is built on the Data Distribution Service (DDS) and leverages more than 20 Quality of Service (QoS) policies to control communication availability, reliability, and resource usage. However, in practice, users often lack clear guidance or pre-verification procedures for combining these policies, which frequently forces them into trial-and-error tuning or results in unexpected runtime failures.
To address this challenge, we decompose DDS publisher–subscriber communication into three phases—Discovery, Data Exchange, and Disassociation—and provide a tutorial-style explanation of how 16 key QoS policies operate at each stage. We also systematically analyze inter-policy dependencies, deriving a QoS Dependency Chain, and classify 40 common constraints into a set of Dependency-Violation Rules.
Building on this analysis, we developed the QoS Guard package, which enables offline verification of DDS XML profiles to detect potential conflicts before deployment. This allows users to safely configure QoS settings without needing to launch a ROS 2 session.
By offering both conceptual insights and a practical tool, this work helps ROS 2 users better understand and manage QoS policies, ultimately improving the reliability of robot communications and the efficiency of resource utilization.

<hr class="hr-grad-center">

<figure style="text-align: center;">
  <img src="images/QoS_guard.png" width="1000px" alt="QoS Guard Overview">
  <figcaption style="font-style: italic; color: #666; margin-top: 10px;">
    QoS Guard Framework
  </figcaption>
</figure>
