#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from typing import Optional

from launch.launch_context import LaunchContext
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def get_xacro_file_content(
    context: LaunchContext,
    *,
    xacro_file=PathJoinSubstitution([  # type: ignore
        FindPackageShare("xarm_description"),
        "urdf/xarm_device.urdf.xacro",
    ]),
    arguments: Optional[dict] = None,
):
    xacro_file: str = xacro_file.perform(context)

    args = {}
    if xacro_file.endswith("urdf/dual_xarm_device.urdf.xacro"):
        raise NotImplementedError("Not yet for dual arm URDF")
        args["hw_ns"] = LaunchConfiguration("hw_ns").perform(context).strip("/")  # type: ignore
    elif xacro_file.endswith("srdf/dual_xarm.srdf.xacro"):
        raise NotImplementedError("Not yet for dual arm SRDF")
    elif xacro_file.endswith("urdf/xarm_device.urdf.xacro"):
        args = {
            arg: LaunchConfiguration(arg)
            for arg in [
                "prefix",
                "hw_ns",
                "limited",
                "effort_control",
                "velocity_control",
                "add_gripper",
                "add_vacuum_gripper",
                "add_bio_gripper",
                "model1300",
                "dof",
                "robot_ip",
                "robot_type",
                "robot_sn",
                "report_type",
                "ros2_control_plugin",
                "ros2_control_params",
                "load_gazebo_plugin",
                "load_ros2_control",
                "add_realsense_d435i",
                "add_realsense_d435i_tilt",
                "add_d435i_links",
                "use_gazebo_camera",
                "add_other_geometry",
                "geometry_type",
                "geometry_mass",
                "geometry_height",
                "geometry_radius",
                "geometry_length",
                "geometry_width",
                "geometry_mesh_filename",
                "geometry_mesh_origin_xyz",
                "geometry_mesh_origin_rpy",
                "geometry_mesh_tcp_xyz",
                "geometry_mesh_tcp_rpy",
                "baud_checkset",
                "default_gripper_baud",
                "attach_to",
                "attach_xyz",
                "attach_rpy",
                "mesh_suffix",
                "kinematics_suffix",
            ]
        }
        args["hw_ns"] = LaunchConfiguration("hw_ns").perform(context).strip("/")  # type: ignore
    elif xacro_file.endswith("srdf/xarm.srdf.xacro"):
        args = {
            arg: LaunchConfiguration(arg)
            for arg in [
                "prefix",
                "dof",
                "robot_type",
                "add_gripper",
                "add_vacuum_gripper",
                "add_bio_gripper",
                "add_other_geometry",
            ]
        }
    else:
        raise NotImplementedError(f"Unknown xacro file: {xacro_file}")

    # Override using arguments
    if arguments is not None:
        for key, val in arguments.items():
            args[key] = val

    command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_file,
        " ",
    ]
    for key, val in args.items():
        command.extend([f"{key}:=", val, " "])
    return Command(command)
