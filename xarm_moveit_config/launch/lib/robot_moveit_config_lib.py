#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from typing import Optional

import yaml
from ament_index_python import get_package_share_directory
from launch.launch_context import LaunchContext
from launch.launch_description_sources import load_python_launch_file_as_module
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, *file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *file_path)
    if not os.path.exists(absolute_file_path):
        return {}
    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return {}
    except Exception:
        return {}


def add_prefix_to_moveit_params(
    controllers_yaml=None,
    ompl_planning_yaml=None,
    kinematics_yaml=None,
    joint_limits_yaml=None,
    prefix="",
):
    if not prefix:
        return
    if controllers_yaml:
        for i, name in enumerate(controllers_yaml["controller_names"]):
            joints = controllers_yaml.get(name, {}).get("joints", [])
            for j, joint in enumerate(joints):
                joints[j] = f"{prefix}{joint}"
            controllers_yaml["controller_names"][i] = f"{prefix}{name}"
            if name in controllers_yaml:
                controllers_yaml[f"{prefix}{name}"] = controllers_yaml.pop(name)
    if ompl_planning_yaml:
        for name in list(ompl_planning_yaml.keys()):
            if name != "planner_configs":
                ompl_planning_yaml[f"{prefix}{name}"] = ompl_planning_yaml.pop(name)
    if kinematics_yaml:
        for name in list(kinematics_yaml.keys()):
            kinematics_yaml[f"{prefix}{name}"] = kinematics_yaml.pop(name)
    if joint_limits_yaml:
        for name in list(joint_limits_yaml["joint_limits"]):
            joint_limits_yaml["joint_limits"][f"{prefix}{name}"] = joint_limits_yaml[
                "joint_limits"
            ].pop(name)


def get_xarm_robot_description_parameters(
    context: LaunchContext,
    xarm_type: str,
    *,
    xacro_urdf_file=PathJoinSubstitution([
        FindPackageShare("xarm_description"),
        "urdf/xarm_device.urdf.xacro",
    ]),
    xacro_srdf_file=PathJoinSubstitution([
        FindPackageShare("xarm_moveit_config"),
        "srdf/xarm.srdf.xacro",
    ]),
    urdf_arguments: Optional[dict] = None,
    srdf_arguments: Optional[dict] = None,
):
    """
    :param xarm_type: for xarm_moveit_config/config/<xarm_type>/*.yaml
    """
    # xarm_description/launch/lib/robot_description_lib.py
    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory("xarm_description"),
            "launch/lib/robot_description_lib.py",
        )
    )

    return {
        "robot_description": mod.get_xacro_file_content(
            context, xacro_file=xacro_urdf_file, arguments=urdf_arguments
        ),
        "robot_description_semantic": mod.get_xacro_file_content(
            context, xacro_file=xacro_srdf_file, arguments=srdf_arguments
        ),
        "robot_description_planning": load_yaml(
            "xarm_moveit_config", f"config/{xarm_type}/joint_limits.yaml"
        ),
        "robot_description_kinematics": load_yaml(
            "xarm_moveit_config", f"config/{xarm_type}/kinematics.yaml"
        ),
    }
