#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import json
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    load_python_launch_file_as_module,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    dof = LaunchConfiguration("dof").perform(context)

    add_gripper = LaunchConfiguration("add_gripper").perform(context)
    node_executable = LaunchConfiguration("node_executable")
    node_name = LaunchConfiguration("node_name", default=node_executable)
    node_parameters = LaunchConfiguration("node_parameters").perform(context)
    use_gripper_node = LaunchConfiguration(
        "use_gripper_node", default=add_gripper
    ).perform(context)

    moveit_config_package_name = "xarm_moveit_config"
    xarm_type = "{}{}".format(robot_type, dof if robot_type in ("xarm", "lite") else "")

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/robot_moveit_config_lib.py
    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory(moveit_config_package_name),
            "launch/lib/robot_moveit_config_lib.py",
        )
    )
    robot_description_parameters = mod.get_xarm_robot_description_parameters(
        context, xarm_type
    )
    kinematics_yaml = robot_description_parameters["robot_description_kinematics"]
    joint_limits_yaml = robot_description_parameters.get(
        "robot_description_planning", None
    )
    mod.add_prefix_to_moveit_params(
        kinematics_yaml=kinematics_yaml,
        joint_limits_yaml=joint_limits_yaml,
        prefix=prefix,
    )
    try:
        xarm_planner_parameters = json.loads(node_parameters)
    except Exception:
        xarm_planner_parameters = {}

    xarm_planner_node = Node(
        name=node_name,
        package="xarm_planner",
        executable=node_executable,
        output="screen",
        parameters=[
            robot_description_parameters,
            {"robot_type": robot_type, "dof": dof, "prefix": prefix},
            xarm_planner_parameters,
        ],
    )

    nodes = [xarm_planner_node]
    if (
        robot_type != "lite"
        and add_gripper in ("True", "true")
        and use_gripper_node in ("True", "true")
    ):
        planning_group = "uf850_gripper" if robot_type == "uf850" else "xarm_gripper"
        xarm_gripper_planner_node = Node(
            name="xarm_gripper_planner_node",
            package="xarm_planner",
            executable="xarm_gripper_planner_node",
            output="screen",
            parameters=[
                robot_description_parameters,
                {"PLANNING_GROUP": planning_group},
            ],
        )
        nodes.append(xarm_gripper_planner_node)
    return nodes


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            name="node_executable",
            default_value="xarm_planner_node",
            description="Node executable name",  # TODO: what is this?
        ),
        DeclareLaunchArgument(
            name="node_name",
            default_value="xarm_planner_node",
            description="Node name",  # TODO: what is this?
        ),
        DeclareLaunchArgument(
            name="node_parameters",
            default_value="{}",
            description="Node parameters",  # TODO: what is this?
        ),
        DeclareLaunchArgument(
            name="use_gripper_node",
            default_value="False",
            description="Use gripper node or not",  # TODO: what is this?
        ),
    ]

    # Declared xarm_device arguments launch
    xarm_device_args_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch/_xarm_device_args.launch.py",
            ])
        ),
        launch_arguments={
            "limited": LaunchConfiguration("limited", default=True),
            "ros2_control_plugin": LaunchConfiguration(
                "ros2_control_plugin",
                default="uf_robot_hardware/UFRobotFakeSystemHardware",
            ),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [xarm_device_args_launch, OpaqueFunction(function=launch_setup)]
    )
