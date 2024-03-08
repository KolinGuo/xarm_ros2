#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # robot joint state launch
    # xarm_description/launch/_robot_joint_state.launch.py
    robot_joint_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch/_robot_joint_state.launch.py",
            ])
        ]),
    )

    # rviz2 display launch
    # xarm_description/launch/_rviz_display.launch.py
    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch/_rviz_display.launch.py",
            ]),
        ]),
    )

    return LaunchDescription([robot_joint_state_launch, rviz2_launch])
