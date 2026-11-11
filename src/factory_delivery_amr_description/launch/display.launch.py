#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = "factory_delivery_amr_description"

    package_share = get_package_share_directory(
        package_name
    )

    default_xacro_file = os.path.join(
        package_share,
        "urdf",
        "factory_delivery_amr.urdf.xacro",
    )

    default_rviz_file = os.path.join(
        package_share,
        "rviz",
        "display.rviz",
    )


    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    model_argument = DeclareLaunchArgument(
        name="model",
        default_value=default_xacro_file,
        description="Absolute path to the robot Xacro file",
    )

    use_sim_time_argument = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )


    # ========================================================================
    # ROBOT DESCRIPTION
    # ========================================================================

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
            ]
        ),
        value_type=str,
    )


    # ========================================================================
    # ROBOT STATE PUBLISHER
    #
    # Reads:
    #   /robot_description
    #   /joint_states
    #
    # Publishes:
    #   /tf
    #   /tf_static
    #
    # The four scissor joints are defined as real URDF joints.
    # Their mimic tags make them follow lift_joint automatically in RViz2.
    # ========================================================================

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ),
            }
        ],
    )


    # ========================================================================
    # JOINT STATE PUBLISHER GUI
    #
    # Main adjustable joints:
    #   left_wheel_joint
    #   right_wheel_joint
    #   lift_joint
    #
    # The scissor joints follow lift_joint through URDF mimic tags.
    # Do not run scissor_visualizer.py in this launch file.
    # ========================================================================

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ),
            }
        ],
    )


    # ========================================================================
    # RVIZ2
    # ========================================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_file],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ),
            }
        ],
    )


    # ========================================================================
    # COMPLETE LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription(
        [
            model_argument,
            use_sim_time_argument,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )