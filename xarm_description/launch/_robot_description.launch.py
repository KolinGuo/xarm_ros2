#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    load_python_launch_file_as_module,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory("xarm_description"),
            "launch/lib/robot_description_lib.py",
        )
    )
    robot_description = {"robot_description": mod.get_xacro_file_content(context)}

    # robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )
    return [robot_state_publisher_node]


def generate_launch_description():
    # Declared xarm_device arguments launch
    xarm_device_args_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch/_xarm_device_args.launch.py",
            ])
        )
    )

    return LaunchDescription([
        xarm_device_args_launch,
        OpaqueFunction(function=launch_setup),
    ])
