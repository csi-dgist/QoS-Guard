#!/usr/bin/env python3
"""
QoS-Guard static analysis test complete launch
- Topic A: cmd_vel 1:1
- Topic B: sensor_data 1:3 (1 pub, 3 subs)
- Topic C: map N:1 (2 pubs, 1 sub)
- Topic D: status isolated (pub, sub each without connection)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Topic A: /cmd_vel 1:1
    ld.add_action(
        Node(
            package="qos_test_pkg",
            executable="cmd_vel_pub",
            name="cmd_vel_pub",
        )
    )
    ld.add_action(
        Node(
            package="qos_test_pkg",
            executable="cmd_vel_sub",
            name="cmd_vel_sub",
        )
    )

    # Topic B: /sensor_data 1:3
    ld.add_action(
        Node(
            package="qos_test_pkg",
            executable="sensor_pub",
            name="sensor_pub",
        )
    )
    for i in range(1, 4):
        ld.add_action(
            Node(
                package="qos_test_pkg",
                executable="sensor_sub",
                name=f"sensor_sub_{i}",
                arguments=[str(i)],
            )
        )

    # Topic C: /map N:1
    ld.add_action(
        Node(
            package="qos_test_pkg",
            executable="map_pub",
            name="map_pub_1",
            arguments=["1"],
        )
    )
    ld.add_action(
        Node(
            package="qos_test_pkg",
            executable="map_pub",
            name="map_pub_2",
            arguments=["2"],
        )
    )
    ld.add_action(
        Node(
            package="qos_test_pkg",
            executable="map_sub",
            name="map_sub",
        )
    )

    # Topic D: /status isolated
    ld.add_action(
        Node(
            package="qos_test_pkg",
            executable="status_pub",
            name="status_pub",
        )
    )
    ld.add_action(
        Node(
            package="qos_test_pkg",
            executable="status_sub",
            name="status_sub",
        )
    )

    return ld
