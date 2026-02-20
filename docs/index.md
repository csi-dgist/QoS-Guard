# QoS Guard

## ROS 2 QoS Static Verifier

**Dependency Chain Analysis of ROS 2 DDS QoS Policies**  
*From Lifecycle Tutorial to Static Verification*

<p align="center">
  <img alt="ROS 2" src="https://img.shields.io/badge/ROS--2-Humble-blue?style=for-the-badge">
  <img alt="Fast DDS" src="https://img.shields.io/badge/Fast--DDS-2.6.9-brightgreen?style=for-the-badge">
</p>

---

## Overview

ROS 2 is built on the **Data Distribution Service (DDS)** and uses more than **20 Quality of Service (QoS)** policies to control communication availability, reliability, and resource usage. In practice, users often lack clear guidance or pre-verification for combining these policies, leading to trial-and-error tuning or unexpected runtime failures.

**QoS Guard** addresses this by providing both **conceptual analysis** and a **practical offline verification tool** for DDS XML profiles—so you can detect conflicts before deployment, without launching a ROS 2 session.

---

## Abstract

!!! abstract "Summary"
    We decompose DDS publisher–subscriber communication into **three phases**—*Discovery*, *Data Exchange*, and *Disassociation*—and explain how **16 key QoS policies** operate at each stage. We analyze inter-policy dependencies into a **QoS Dependency Chain** and classify **40 common constraints** into **Dependency-Violation Rules**.

    The **QoS Guard** package enables offline verification of DDS XML profiles to detect potential conflicts before deployment, helping ROS 2 users understand and manage QoS policies and improve the reliability of robot communications and resource utilization.

---

## Key highlights

:::: grid
::: grid-item
:material-graph-outline: **3 phases**
DDS communication decomposed into Discovery, Data Exchange, and Disassociation.
:::
::: grid-item
:material-format-list-numbered: **16 QoS policies**
Tutorial-style explanation of how key policies operate at each stage.
:::
::: grid-item
:material-shield-check: **40 rules**
Dependency-violation rules for static verification (STD, IMP, EMP).
:::
::: grid-item
:material-file-document-check: **Offline verification**
Check DDS XML profiles without launching a ROS 2 session.
:::
::::

---

## Architecture

<p align="center">
  <img src="images/QoS_guard.png" alt="QoS Guard architecture" width="100%">
</p>

*QoS Guard: Static verification flow from DDS XML profiles to dependency-violation detection.*

---

## Next steps

[:octicons-arrow-right-24: Explore QoS Rules](/rules/) — Browse the 40 dependency-violation rules (STD, IMP, EMP) and their conditions.
