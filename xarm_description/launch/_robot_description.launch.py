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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import load_python_launch_file_as_module
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    robot_description = {
        "robot_description": mod.get_xacro_file_content(
            xacro_file=LaunchConfiguration("xacro_file"),
            arguments={
                arg: LaunchConfiguration(arg).perform(context).strip("/")
                if arg == "hw_ns"
                else LaunchConfiguration(arg)
                for arg in [
                    "prefix",
                    "hw_ns",
                    "limited",
                    "effort_control",
                    "velocity_control",
                    "add_gripper",
                    "add_vacuum_gripper",
                    "add_bio_gripper",
                    "dof",
                    "robot_type",
                    "add_realsense_d435i",
                    "add_realsense_d435i_tilt",
                    "add_d435i_links",
                    "model1300",
                    "robot_sn",
                    "attach_to",
                    "attach_xyz",
                    "attach_rpy",
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
                    "kinematics_suffix",
                    "ros2_control_plugin",
                    "load_gazebo_plugin",
                    "load_ros2_control",
                    # "joint_states_remapping",
                ]
            },
        )
    }

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
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            name="prefix", default_value="", description="Joint/Link name prefix"
        ),
        DeclareLaunchArgument(
            name="hw_ns",
            default_value="xarm",
            description="All the services and topics in xarm_api are "
            "under <hw_ns>/ namespace",
        ),
        DeclareLaunchArgument(
            name="limited", default_value="False", description="Use limited joint range"
        ),
        DeclareLaunchArgument(
            name="effort_control",
            default_value="False",
            description="Use EffortJointInterface for ros2_control",
        ),
        DeclareLaunchArgument(
            name="velocity_control",
            default_value="False",
            description="Use VelocityJointInterface for ros2_control",
        ),
        DeclareLaunchArgument(
            name="add_gripper", default_value="False", description="Add xArm gripper"
        ),
        DeclareLaunchArgument(
            name="add_vacuum_gripper",
            default_value="False",
            description="Add xArm vacuum gripper",
        ),
        DeclareLaunchArgument(
            name="add_bio_gripper",
            default_value="False",
            description="Add xArm BIO gripper",
        ),
        DeclareLaunchArgument(
            name="dof",
            default_value="7",
            description="xArm DOF",
            choices=["7", "6", "5"],
        ),
        DeclareLaunchArgument(
            name="robot_type",
            default_value="xarm",
            description="Robot type",
            choices=["xarm", "uf850", "lite", "xarm7_mirror"],
        ),
        DeclareLaunchArgument(
            name="add_realsense_d435i",
            default_value="False",
            description="Add RealSense D435i camera with camera mount to link_eef",
        ),
        DeclareLaunchArgument(
            name="add_realsense_d435i_tilt",
            default_value="False",
            description="Add RealSense D435i camera with tilted camera mount to "
            "link_eef",
        ),
        DeclareLaunchArgument(
            name="add_d435i_links",
            default_value="True",
            description="Add RealSense D435i links (camera frames)",
        ),
        DeclareLaunchArgument(
            name="model1300",
            default_value="False",
            description="Whether the robot model number (robot_sn[2:6]) is >= 1300. "
            "Affects link7 mesh",
        ),
        DeclareLaunchArgument(
            name="robot_sn",
            default_value="",
            description="Robot serial number to load inertia parameters and "
            "overrides the model1300 parameter",
        ),
        DeclareLaunchArgument(
            name="attach_to",
            default_value="world",
            description="Attach xArm link_base to this link",
        ),
        DeclareLaunchArgument(
            name="attach_xyz",
            default_value='"0 0 0"',
            description="Attaching joint's origin xyz",
        ),
        DeclareLaunchArgument(
            name="attach_rpy",
            default_value='"0 0 0"',
            description="Attaching joint's origin rpy",
        ),
        DeclareLaunchArgument(
            name="add_other_geometry",
            default_value="False",
            description="Add other geometry to link_eef "
            "(if not add_gripper and not add_vacuum_gripper)",
        ),
        DeclareLaunchArgument(
            name="geometry_type",
            default_value="box",
            description="Other geometry type",
            choices=["box", "sphere", "cylinder", "mesh"],
        ),
        DeclareLaunchArgument(
            name="geometry_mass",
            default_value="0.1",
            description="Mass of the inertial property of the other geometry",
        ),
        DeclareLaunchArgument(
            name="geometry_height",
            default_value="0.1",
            description="Height of the other geometry",
        ),
        DeclareLaunchArgument(
            name="geometry_radius",
            default_value="0.1",
            description="Radius of the other geometry",
        ),
        DeclareLaunchArgument(
            name="geometry_length",
            default_value="0.1",
            description="Length of the other geometry",
        ),
        DeclareLaunchArgument(
            name="geometry_width",
            default_value="0.1",
            description="Width of the other geometry",
        ),
        DeclareLaunchArgument(
            name="geometry_mesh_filename",
            default_value="",
            description="Mesh filename of the other geometry",
        ),
        DeclareLaunchArgument(
            name="geometry_mesh_origin_xyz",
            default_value='"0 0 0"',
            description="Mesh origin xyz of the other geometry",
        ),
        DeclareLaunchArgument(
            name="geometry_mesh_origin_rpy",
            default_value='"0 0 0"',
            description="Mesh origin rpy of the other geometry",
        ),
        DeclareLaunchArgument(
            name="geometry_mesh_tcp_xyz",
            default_value='"0 0 0"',
            description="When other geometry_type is mesh, link_tcp's joint origin xyz",
        ),
        DeclareLaunchArgument(
            name="geometry_mesh_tcp_rpy",
            default_value='"0 0 0"',
            description="When other geometry_type is mesh, link_tcp's joint origin rpy",
        ),
        DeclareLaunchArgument(
            name="kinematics_suffix",
            default_value="",
            description="Suffix of kinematics_params_filename",
        ),
        DeclareLaunchArgument(
            name="ros2_control_plugin",
            default_value="uf_robot_hardware/UFRobotSystemHardware",
            description="Plugin for ros2_control",
        ),
        DeclareLaunchArgument(
            name="load_gazebo_plugin",
            default_value="True",
            description="Load gazebo plugin and transmission elements",
        ),
        DeclareLaunchArgument(
            name="load_ros2_control",
            default_value="True",
            description="Load ros2_control.xacro files",
        ),
        # DeclareLaunchArgument(
        #     name="joint_states_remapping", default_value="joint_states"
        # ),
        DeclareLaunchArgument(
            name="xacro_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "urdf/xarm_device.urdf.xacro",
            ]),
            description="Path to the top-level xacro file",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
