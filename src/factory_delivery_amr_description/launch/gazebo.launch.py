#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "factory_delivery_amr_description"

    package_share = get_package_share_directory(
        package_name
    )

    ros_gz_sim_share = get_package_share_directory(
        "ros_gz_sim"
    )

    # ========================================================================
    # RESOURCE PATHS FOR GAZEBO SIM / FORTRESS
    # Ensures Gazebo resolves model:// and package:// meshes accurately
    # ========================================================================

    models_dir = os.path.join(package_share, "models")
    parent_share_dir = os.path.dirname(package_share)

    set_ign_resource_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=f"{models_dir}:{parent_share_dir}",
    )
    set_gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{models_dir}:{parent_share_dir}",
    )


    # ========================================================================
    # FILE PATHS
    # ========================================================================

    xacro_file = os.path.join(
        package_share,
        "urdf",
        "factory_delivery_amr.urdf.xacro",
    )

    world_file = os.path.join(
        package_share,
        "worlds",
        "factory_world.sdf",
    )

    bridge_file = os.path.join(
        package_share,
        "config",
        "bridge.yaml",
    )

    generated_urdf_1 = "/tmp/factory_delivery_amr_1.urdf"
    generated_urdf_2 = "/tmp/factory_delivery_amr_2.urdf"

    # ========================================================================
    # GENERATE URDF FILES FOR ROBOT 1 (/amr_1) AND ROBOT 2 (/amr_2)
    # ========================================================================

    generate_urdf_1 = ExecuteProcess(
        cmd=[
            "xacro",
            xacro_file,
            "topic_prefix:=/amr_1",
            "-o",
            generated_urdf_1,
        ],
        output="screen",
    )

    generate_urdf_2 = ExecuteProcess(
        cmd=[
            "xacro",
            xacro_file,
            "topic_prefix:=/amr_2",
            "-o",
            generated_urdf_2,
        ],
        output="screen",
    )


    # ========================================================================
    # START GAZEBO FORTRESS SERVER AND GUI
    #
    # -r:
    #   Start the simulation immediately.
    #
    # -v 4:
    #   Print detailed Gazebo diagnostic output.
    # ========================================================================

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": (
                "-r -v 4 " +
                world_file
            ),
            "on_exit_shutdown": "true",
        }.items(),
    )


    # ========================================================================
    # ROS 2 TO GAZEBO BRIDGE
    #
    # Topics are configured in:
    # config/bridge.yaml
    #
    # Expected topics:
    # - /clock
    # - /cmd_vel
    # - /odom
    # - /lift_position
    # - /scissor_left_a_position
    # - /scissor_left_b_position
    # - /scissor_right_a_position
    # - /scissor_right_b_position
    # ========================================================================

    bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": bridge_file,
                        "use_sim_time": True,
                    }
                ],
            )
        ],
    )


    # ========================================================================
    # SPAWN TWO ROBOTS (amr_1 & amr_2) INTO GAZEBO
    #   - amr_1 at (0.0, +0.8, 0.01)
    #   - amr_2 at (0.0, -0.8, 0.01)
    # ========================================================================

    spawn_robot_1 = TimerAction(
        period=6.5,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_factory_delivery_amr_1",
                output="screen",
                arguments=[
                    "-world",
                    "factory_world",
                    "-file",
                    generated_urdf_1,
                    "-name",
                    "amr_1",
                    "-x",
                    "0.0",
                    "-y",
                    "0.80",
                    "-z",
                    "0.01",
                ],
            )
        ],
    )

    spawn_robot_2 = TimerAction(
        period=7.5,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_factory_delivery_amr_2",
                output="screen",
                arguments=[
                    "-world",
                    "factory_world",
                    "-file",
                    generated_urdf_2,
                    "-name",
                    "amr_2",
                    "-x",
                    "0.0",
                    "-y",
                    "-0.80",
                    "-z",
                    "0.01",
                ],
            )
        ],
    )

    # ========================================================================
    # LIFT AND SCISSOR SYNCHRONIZATION CONTROLLERS FOR BOTH ROBOTS
    # ========================================================================

    lift_controller_amr_1 = TimerAction(
        period=9.0,
        actions=[
            Node(
                package=package_name,
                executable="lift_gazebo_controller.py",
                name="lift_gazebo_controller_amr_1",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "topic_prefix": "/amr_1",
                    }
                ],
            )
        ],
    )

    lift_controller_amr_2 = TimerAction(
        period=9.2,
        actions=[
            Node(
                package=package_name,
                executable="lift_gazebo_controller.py",
                name="lift_gazebo_controller_amr_2",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "topic_prefix": "/amr_2",
                    }
                ],
            )
        ],
    )

    # ========================================================================
    # COMPLETE LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription(
        [
            set_ign_resource_path,
            set_gz_resource_path,
            generate_urdf_1,
            generate_urdf_2,
            start_gazebo,
            bridge,
            spawn_robot_1,
            spawn_robot_2,
            lift_controller_amr_1,
            lift_controller_amr_2,
        ]
    )