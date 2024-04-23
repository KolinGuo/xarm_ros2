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
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
    add_bio_gripper = LaunchConfiguration("add_bio_gripper").perform(context)

    no_gui_ctrl = LaunchConfiguration("no_gui_ctrl").perform(context)
    controllers_name = LaunchConfiguration("controllers_name").perform(context)
    moveit_controller_manager_key = LaunchConfiguration(
        "moveit_controller_manager_key"
    ).perform(context)
    moveit_controller_manager_value = LaunchConfiguration(
        "moveit_controller_manager_value"
    ).perform(context)

    attach_xyz = LaunchConfiguration("attach_xyz").perform(context)
    attach_rpy = LaunchConfiguration("attach_rpy").perform(context)

    use_sim_time = LaunchConfiguration("use_sim_time")

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

    controllers_yaml = mod.load_yaml(
        moveit_config_package_name, f"config/{xarm_type}/{controllers_name}.yaml"
    )
    ompl_planning_yaml = mod.load_yaml(
        moveit_config_package_name, f"config/{xarm_type}/ompl_planning.yaml"
    )
    kinematics_yaml = robot_description_parameters["robot_description_kinematics"]
    joint_limits_yaml = robot_description_parameters.get(
        "robot_description_planning", None
    )

    if add_gripper in ("True", "true"):
        gripper_controllers_yaml = mod.load_yaml(
            moveit_config_package_name,
            f"config/{robot_type}_gripper/{controllers_name}.yaml",
        )
        gripper_ompl_planning_yaml = mod.load_yaml(
            moveit_config_package_name,
            f"config/{robot_type}_gripper/ompl_planning.yaml",
        )
        gripper_joint_limits_yaml = mod.load_yaml(
            moveit_config_package_name, f"config/{robot_type}_gripper/joint_limits.yaml"
        )

        if gripper_controllers_yaml and "controller_names" in gripper_controllers_yaml:
            for name in gripper_controllers_yaml["controller_names"]:
                if name in gripper_controllers_yaml:
                    if name not in controllers_yaml["controller_names"]:
                        controllers_yaml["controller_names"].append(name)
                    controllers_yaml[name] = gripper_controllers_yaml[name]
        if gripper_ompl_planning_yaml:
            ompl_planning_yaml.update(gripper_ompl_planning_yaml)
        if joint_limits_yaml and gripper_joint_limits_yaml:
            joint_limits_yaml["joint_limits"].update(
                gripper_joint_limits_yaml["joint_limits"]
            )
    elif add_bio_gripper in ("True", "true"):
        gripper_controllers_yaml = mod.load_yaml(
            moveit_config_package_name, f"config/bio_gripper/{controllers_name}.yaml"
        )
        gripper_ompl_planning_yaml = mod.load_yaml(
            moveit_config_package_name, "config/bio_gripper/ompl_planning.yaml"
        )
        gripper_joint_limits_yaml = mod.load_yaml(
            moveit_config_package_name, "config/bio_gripper/joint_limits.yaml"
        )

        if gripper_controllers_yaml and "controller_names" in gripper_controllers_yaml:
            for name in gripper_controllers_yaml["controller_names"]:
                if name in gripper_controllers_yaml:
                    if name not in controllers_yaml["controller_names"]:
                        controllers_yaml["controller_names"].append(name)
                    controllers_yaml[name] = gripper_controllers_yaml[name]
        if gripper_ompl_planning_yaml:
            ompl_planning_yaml.update(gripper_ompl_planning_yaml)
        if joint_limits_yaml and gripper_joint_limits_yaml:
            joint_limits_yaml["joint_limits"].update(
                gripper_joint_limits_yaml["joint_limits"]
            )

    mod.add_prefix_to_moveit_params(
        controllers_yaml=controllers_yaml,
        ompl_planning_yaml=ompl_planning_yaml,
        kinematics_yaml=kinematics_yaml,
        joint_limits_yaml=joint_limits_yaml,
        prefix=prefix,
    )

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
    }
    if os.environ.get("ROS_DISTRO", "") > "iron":
        ompl_planning_pipeline_config["ompl"] = {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        }
    else:
        ompl_planning_pipeline_config["ompl"] = {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",  # noqa: E501
            "start_state_max_bounds_error": 0.1,
        }
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Moveit controllers Configuration
    moveit_controllers = {
        moveit_controller_manager_key: controllers_yaml,
        "moveit_controller_manager": moveit_controller_manager_value,
    }

    # Trajectory Execution Configuration
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    plan_execution = {
        "plan_execution.record_trajectory_state_frequency": 10.0,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # "planning_scene_monitor_options": {
        #     "name": "planning_scene_monitor",
        #     "robot_description": "robot_description",
        #     "joint_state_topic": "/joint_states",
        #     "attached_collision_object_topic": "/move_group/planning_scene_monitor",
        #     "publish_planning_scene_topic": "/move_group/publish_planning_scene",
        #     "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
        #     "wait_for_initial_state_timeout": 10.0,
        # },
    }

    # sensor_manager_parameters = {
    #     'sensors': ['ros'],
    #     'octomap_resolution': 0.02,
    #     'ros.sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater',
    #     'ros.point_cloud_topic': '/camera/depth/color/points',
    #     'ros.max_range': 2.0,
    #     'ros.point_subsample': 1,
    #     'ros.padding_offset': 0.1,
    #     'ros.padding_scale': 1.0,
    #     'ros.max_update_rate': 1.0,
    #     'ros.filtered_cloud_topic': 'filtered_cloud',
    # }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_parameters,
            ompl_planning_pipeline_config,
            trajectory_execution,
            plan_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # sensor_manager_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution([
    #    FindPackageShare(moveit_config_package_name),
    #    "config",
    #    xarm_type,
    #    "planner.rviz" if no_gui_ctrl == "true" else "moveit.rviz",
    # ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(moveit_config_package_name),
        "rviz",
        "planner.rviz" if no_gui_ctrl == "true" else "moveit.rviz",
    ])
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description_parameters,
            ompl_planning_pipeline_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    xyz = attach_xyz[1:-1].split(" ")
    rpy = attach_rpy[1:-1].split(" ")
    args = xyz + rpy + ["world", f"{prefix}link_base"]

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        # arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'link_base'],
        arguments=args,
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz2_node, on_exit=[EmitEvent(event=Shutdown())]
            )
        ),
        rviz2_node,
        static_tf,
        move_group_node,
    ]


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            name="no_gui_ctrl",
            default_value="False",
            description="Turn off GUI control",
        ),
        DeclareLaunchArgument(
            name="controllers_name",
            default_value="fake_controllers",
            description="Controllers name",  # TODO: what is this?
        ),
        DeclareLaunchArgument(
            name="moveit_controller_manager_key",
            default_value="moveit_fake_controller_manager",
            description="MoveIt controller manager key",  # TODO: what is this?
        ),
        DeclareLaunchArgument(
            name="moveit_controller_manager_value",
            default_value="moveit_fake_controller_manager/MoveItFakeControllerManager",
            description="MoveIt controller manager value",  # TODO: what is this?
        ),
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="False",
            description="Use simulation time",
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
