#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from tracetools_launch.action import Trace


def generate_launch_description():

    args = [
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        DeclareLaunchArgument("trace", default_value="true", description="enable tracing"),
        DeclareLaunchArgument("dummy_publisher", default_value="true", description="enable dummy publisher"),
        DeclareLaunchArgument("test_subscriber_1", default_value="true", description="enable test subscriber 1"),
        DeclareLaunchArgument("test_subscriber_2", default_value="true", description="enable test subscriber 2"),
        DeclareLaunchArgument("duration", default_value="0", description="Duration in seconds after which to stop all nodes (0 = run indefinitely)"),
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
            condition=IfCondition(LaunchConfiguration("dummy_publisher")),
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
            condition=IfCondition(LaunchConfiguration("test_subscriber_1")),
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
            condition=IfCondition(LaunchConfiguration("test_subscriber_2")),
        ),
        Trace(
            session_name='trace',
            condition=IfCondition(LaunchConfiguration("trace")),
        ),
    ]

    # Timer to automatically shutdown after specified duration
    shutdown_timer = TimerAction(
        period=LaunchConfiguration("duration"),
        actions=[Shutdown(reason='Duration timeout reached')]
    )

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        *nodes,
        shutdown_timer,
    ])
