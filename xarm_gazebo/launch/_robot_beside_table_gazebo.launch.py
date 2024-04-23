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
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    load_python_launch_file_as_module,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix", default="")
    add_gripper = LaunchConfiguration("add_gripper", default=False)
    add_bio_gripper = LaunchConfiguration("add_bio_gripper", default=False)
    dof = LaunchConfiguration("dof", default=7)
    robot_type = LaunchConfiguration("robot_type", default="xarm")

    load_controller = LaunchConfiguration("load_controller", default=False)

    ros_namespace = LaunchConfiguration("ros_namespace", default="").perform(context)

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
        update_rate=1000,
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

    # robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    xarm_gazebo_world = PathJoinSubstitution([
        FindPackageShare("xarm_gazebo"),
        "worlds/table.world",
    ])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch/gazebo.launch.py",
            ])
        ),
        launch_arguments={
            "world": xarm_gazebo_world,
            "server_required": "true",
            "gui_required": "true",
        }.items(),
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            # '-entity', '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else ''),
            "-entity",
            "UF_ROBOT",
            "-x",
            "-0.2",
            "-y",
            "-0.54" if robot_type.perform(context) == "uf850" else "-0.5",
            "-z",
            "1.021",
            "-Y",
            "1.571",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Load controllers
    controllers = [
        "joint_state_broadcaster",
        "{}{}{}_traj_controller".format(
            prefix.perform(context),
            robot_type.perform(context),
            dof.perform(context)
            if robot_type.perform(context) in ("xarm", "lite")
            else "",
        ),
    ]
    if robot_type.perform(context) != "lite" and add_gripper.perform(context) in (
        "True",
        "true",
    ):
        controllers.append(
            "{}{}_gripper_traj_controller".format(
                prefix.perform(context), robot_type.perform(context)
            )
        )
    elif robot_type.perform(context) != "lite" and add_bio_gripper.perform(context) in (
        "True",
        "true",
    ):
        controllers.append(
            "{}bio_gripper_traj_controller".format(prefix.perform(context))
        )
    load_controllers = []
    if load_controller.perform(context) in ("True", "true"):
        for controller in controllers:
            load_controllers.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    output="screen",
                    arguments=[
                        controller,
                        "--controller-manager",
                        "{}/controller_manager".format(ros_namespace),
                    ],
                    parameters=[{"use_sim_time": True}],
                )
            )

    if len(load_controllers) > 0:
        return [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=load_controllers,
                )
            ),
            gazebo_launch,
            robot_state_publisher_node,
            gazebo_spawn_entity_node,
        ]
    else:
        return [
            gazebo_launch,
            robot_state_publisher_node,
            gazebo_spawn_entity_node,
        ]


def generate_launch_description():
    # Declared xarm_device arguments launch
    xarm_device_args_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch/_xarm_device_args.launch.py",
            ])
        ),
        launch_arguments={
            "ros2_control_plugin": LaunchConfiguration(
                "ros2_control_plugin", default="gazebo_ros2_control/GazeboSystem"
            ),
        }.items(),
    )

    return LaunchDescription([
        xarm_device_args_launch,
        OpaqueFunction(function=launch_setup),
    ])
