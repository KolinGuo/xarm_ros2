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
    # robot rviz launch
    # xarm_description/launch/_robot_rviz_display.launch.py
    robot_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch/_robot_rviz_display.launch.py",
            ])
        ),
        launch_arguments={
            "dof": "7",
            "robot_type": "xarm",
        }.items(),
    )
    return LaunchDescription([robot_rviz_launch])
