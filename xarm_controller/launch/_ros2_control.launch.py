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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix", default="")
    robot_type = LaunchConfiguration("robot_type", default="xarm")
    add_gripper = LaunchConfiguration("add_gripper", default=False)
    add_bio_gripper = LaunchConfiguration("add_bio_gripper", default=False)
    dof = LaunchConfiguration("dof", default=7)

    # ros2 control params
    # xarm_controller/launch/lib/robot_controller_lib.py
    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory("xarm_controller"),
            "launch/lib/robot_controller_lib.py",
        )
    )
    ros2_control_params = mod.generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory("xarm_controller"),
            "config",
            "{}{}_controllers.yaml".format(
                robot_type.perform(context),
                dof.perform(context)
                if robot_type.perform(context) in ("xarm", "lite")
                else "",
            ),
        ),
        prefix=prefix.perform(context),
        add_gripper=add_gripper.perform(context) in ("True", "true"),
        add_bio_gripper=add_bio_gripper.perform(context) in ("True", "true"),
        ros_namespace=LaunchConfiguration("ros_namespace", default="").perform(context),
        robot_type=robot_type.perform(context),
    )

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory("xarm_description"),
            "launch/lib/robot_description_lib.py",
        )
    )
    robot_description = {
        "robot_description": mod.get_xacro_file_content(
            context, arguments={"ros2_control_params": ros2_control_params}
        )
    }

    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory("xarm_api"), "launch/lib/robot_api_lib.py"
        )
    )
    robot_params = mod.generate_robot_api_params(
        os.path.join(
            get_package_share_directory("xarm_api"), "config/xarm_params.yaml"
        ),
        os.path.join(
            get_package_share_directory("xarm_api"), "config/xarm_user_params.yaml"
        ),
        LaunchConfiguration("ros_namespace", default="").perform(context),
        node_name="ufactory_driver",
    )

    # ros2 control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_control_params,
            robot_params,
        ],
        output="screen",
    )

    return [ros2_control_node]


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
