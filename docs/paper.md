# Paper

<p align="center">
  <strong>Dependency Chain Analysis of ROS 2 DDS QoS Policies: <br>From Lifecycle Tutorial to Static Validation</strong>
</p>

<p align="center">
  Sanghoon Lee, Junha Kang, and Kyung-Joon Park
</p>

## Abstract

<hr class="hr-grad-center">

ROS 2 relies on the Data Distribution Service (DDS), which offers more than 20 Quality-of-Service (QoS) policies governing availability, reliability, and resource utilization. Yet ROS 2 users lack clear guidance on safe policy combinations and validation processes prior to deployment, which often leads to trial-and-error tuning and unexpected runtime failures.
To address these challenges, we analyze DDS publisher-subscriber communication over a lifecycle divided into Discovery, Data Exchange, and Disassociation, and provide a user-oriented tutorial explaining how 16 QoS policies operate across these phases. We derive a QoS Policy Chain that encodes inter-policy relationships as 40 dependency-violation rules grounded in the OMG DDS standard, open-source implementations, and empirical measurements.
Finally, we introduce QoS Guard, a ROS 2 tool that statically extracts endpoint QoS settings from a package, validates them offline, flags conflicts, and enables pre-deployment risk detection without establishing a live ROS 2 session. Together, these contributions give ROS 2 users both conceptual insight and a concrete tool that enables early detection of misconfigurations, improving the reliability and resource efficiency of ROS 2-based robotic systems.

<hr class="hr-grad-center">

## Related site sections

- [Home](index.md): QoS Guard overview and usage
- [QoS Encyclopedia](qos/index.md): definitions and lifecycle roles of DDS QoS policies
- [Dependency Map](chain.md): interactive dependency graph and relationship overview
- [Rules & Evidence](rules/index.md): the 40 rules with Fast DDS and Cyclone DDS verification, each linked to its evidence page
