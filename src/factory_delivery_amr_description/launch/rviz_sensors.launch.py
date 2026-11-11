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

    xacro_file = os.path.join(
        package_share,
        "urdf",
        "factory_delivery_amr.urdf.xacro",
    )

    rviz_config_file = os.path.join(
        package_share,
        "rviz",
        "amr_gazebo_sensors.rviz",
    )

    # ========================================================================
    # ROBOT DESCRIPTION (amr_1)
    # ========================================================================

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " topic_prefix:=/amr_1",
            ]
        ),
        value_type=str,
    )

    # ========================================================================
    # ROBOT STATE PUBLISHER (amr_1)
    # Publishes static and dynamic joint transforms into /tf and /tf_static
    # ========================================================================

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_amr_1",
        namespace="amr_1",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "frame_prefix": "amr_1/",
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
        ],
    )

    # ========================================================================
    # RVIZ2 WITH PRE-CONFIGURED SENSOR DASHBOARD
    # ========================================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
        ]
    )
