#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from tracetools_launch.action import Trace


def generate_launch_description():

    args = [
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        DeclareLaunchArgument("trace", default_value="true", description="enable tracing"),
    ]

    nodes = [
        Node(
            package="test_publisher",
            executable="dummy_publisher",
            namespace="",
            name="dummy_publisher",
            remappings=[
                ("~/topic", "input_topic_1"),
            ],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="test_publisher",
            executable="test_subscriber",
            namespace="",
            name="test_subscriber_1",
            parameters=[],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[
                ("~/input", "input_topic_1"),
                ("~/output", "input_topic_2"),
            ],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="test_publisher",
            executable="test_subscriber",
            namespace="",
            name="test_subscriber_2",
            parameters=[],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[
                ("~/input", "input_topic_2"),
                ("~/output", "output_topic_2"),
            ],
            output="screen",
            emulate_tty=True,
        ),
        Trace(
            session_name='trace',
            condition=IfCondition(LaunchConfiguration("trace")),
        ),
    ]

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        *nodes,
    ])
