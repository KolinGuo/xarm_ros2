#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # joint state publisher node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[
            {
                "source_list": [
                    "{}{}/joint_states".format(
                        LaunchConfiguration("prefix").perform(context),
                        LaunchConfiguration("hw_ns").perform(context),
                    )
                ]
            }
        ],
    )
    return [joint_state_publisher_gui_node]


def generate_launch_description():
    # robot description launch
    # xarm_description/launch/_robot_description.launch.py
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch/_robot_description.launch.py",
            ])
        ),
    )

    return LaunchDescription([
        robot_description_launch,
        OpaqueFunction(function=launch_setup),
    ])
