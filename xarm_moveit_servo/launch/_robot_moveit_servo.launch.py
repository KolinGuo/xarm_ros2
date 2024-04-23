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
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    load_python_launch_file_as_module,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    dof = LaunchConfiguration("dof").perform(context)

    add_gripper = LaunchConfiguration("add_gripper").perform(context)
    add_bio_gripper = LaunchConfiguration("add_bio_gripper").perform(context)

    moveit_config_package_name = "xarm_moveit_config"
    xarm_type = "{}{}".format(robot_type, dof if robot_type in ("xarm", "lite") else "")
    ros_namespace = LaunchConfiguration("ros_namespace").perform(context)

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

    servo_yaml = mod.load_yaml(
        "xarm_moveit_servo", "config/xarm_moveit_servo_config.yaml"
    )
    servo_yaml["move_group_name"] = xarm_type
    xarm_traj_controller = f"{prefix}{xarm_type}_traj_controller"
    servo_yaml["command_out_topic"] = f"/{xarm_traj_controller}/joint_trajectory"
    servo_params = {"moveit_servo": servo_yaml}
    controllers = ["joint_state_broadcaster", xarm_traj_controller]
    if add_gripper in ("True", "true") and robot_type != "lite":
        controllers.append(f"{prefix}{robot_type}_gripper_traj_controller")
    elif add_bio_gripper in ("True", "true") and robot_type != "lite":
        controllers.append(f"{prefix}bio_gripper_traj_controller")

    # rviz_config_file = PathJoinSubstitution([
    #     FindPackageShare(moveit_config_package_name),
    #     "rviz",
    #     "moveit.rviz",
    # ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("xarm_moveit_servo"),
        "rviz/servo.rviz",
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_controller"),
                "launch/_ros2_control.launch.py",
            ])
        )
    )

    # Load controllers
    load_controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                controller,
                "--controller-manager",
                f"{ros_namespace}/controller_manager",
            ],
        )
        for controller in controllers
    ]

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="xarm_moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[robot_description_parameters],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "link_base", "frame_id": "world"}],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_server",
                parameters=[servo_params, robot_description_parameters],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="xarm_moveit_servo",
                plugin="xarm_moveit_servo::JoyToServoPub",
                name="joy_to_servo_node",
                parameters=[
                    servo_params,
                    {
                        "dof": dof,
                        "ros_queue_size": 10,
                        "joystick_type": LaunchConfiguration("joystick_type"),
                        "joy_topic": LaunchConfiguration("joy_topic"),
                    },
                ],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                parameters=[
                    # {'autorepeat_rate': 50.0},
                ],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output="screen",
    )

    return [rviz_node, ros2_control_launch, container] + load_controllers


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            name="joystick_type",
            default_value="1",
            description="Joystick type "
            "(1: xbox360 wired, 2: xbox360 wireless, 3: spacemouse wireless)",
        ),
        DeclareLaunchArgument(
            name="joy_topic",
            default_value="/joy",
            description="Joystick sensor_msgs/msg/Joy topic",
        ),
        DeclareLaunchArgument(
            name="ros_namespace",
            default_value="",
            description="ROS namespace for controller_manager",
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
