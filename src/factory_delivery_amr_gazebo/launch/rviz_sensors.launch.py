#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declare_launch_rsp = DeclareLaunchArgument(
        "launch_rsp",
        default_value="false",
        description="Launch robot_state_publisher nodes here if not started by gazebo.launch.py",
    )
    declare_static_world_tf = DeclareLaunchArgument(
        "use_static_world_tf",
        default_value="true",
        description="Publish static world->odom transforms (disable when SLAM/AMCL is active)",
    )
    declare_multi_robot = DeclareLaunchArgument(
        "multi_robot",
        default_value="false",
        description="Match dual-robot spawn offsets (y=+-0.80) if true, else single robot at (0,0)",
    )

    launch_rsp = LaunchConfiguration("launch_rsp")
    use_static_world_tf = LaunchConfiguration("use_static_world_tf")
    multi_robot = LaunchConfiguration("multi_robot")
    package_name = "factory_delivery_amr_gazebo"
    description_package_name = "factory_delivery_amr_description"

    package_share = get_package_share_directory(
        package_name
    )

    description_share = get_package_share_directory(
        description_package_name
    )

    xacro_file = os.path.join(
        description_share,
        "urdf",
        "factory_delivery_amr.urdf.xacro",
    )

    rviz_config_file = os.path.join(
        package_share,
        "rviz",
        "amr_gazebo_sensors.rviz",
    )

    # ========================================================================
    # ROBOT DESCRIPTIONS FOR ROBOT 1 (/amr_1) AND ROBOT 2 (/amr_2)
    # ========================================================================

    robot_description_1 = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " topic_prefix:=/amr_1",
            ]
        ),
        value_type=str,
    )

    robot_description_2 = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " topic_prefix:=/amr_2",
            ]
        ),
        value_type=str,
    )

    # ========================================================================
    # ROBOT STATE PUBLISHER NODES
    # Publishes static and joint transforms for both robots
    # ========================================================================

    rsp_node_1 = Node(
        condition=IfCondition(launch_rsp),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_amr_1",
        namespace="amr_1",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_1,
                "frame_prefix": "amr_1/",
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
        ],
    )

    rsp_node_2 = Node(
        condition=IfCondition(
            PythonExpression(
                ["'", launch_rsp, "'.lower() == 'true' and '", multi_robot, "'.lower() == 'true'"]
            )
        ),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_amr_2",
        namespace="amr_2",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_2,
                "frame_prefix": "amr_2/",
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
        ],
    )

    # ========================================================================
    # STATIC TRANSFORMS FROM WORLD TO EACH ROBOT'S ODOM FRAME
    # Places robots at their respective spawn positions in RViz space:
    #   - Single robot (multi_robot:=false): amr_1 at (0.0, 0.0, 0.0)
    #   - Dual robot (multi_robot:=true):    amr_1 at (0.0, +0.8), amr_2 at (0.0, -0.8)
    # ========================================================================

    static_tf_world_to_odom_1_single = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_static_world_tf,
                    "'.lower() == 'true' and '",
                    multi_robot,
                    "'.lower() != 'true'",
                ]
            )
        ),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_odom_1",
        arguments=["0", "0", "0", "0", "0", "0", "world", "amr_1/odom"],
        parameters=[{"use_sim_time": True}],
    )

    static_tf_world_to_odom_1_dual = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_static_world_tf,
                    "'.lower() == 'true' and '",
                    multi_robot,
                    "'.lower() == 'true'",
                ]
            )
        ),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_odom_1",
        arguments=["0", "0.80", "0", "0", "0", "0", "world", "amr_1/odom"],
        parameters=[{"use_sim_time": True}],
    )

    static_tf_world_to_odom_2 = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_static_world_tf,
                    "'.lower() == 'true' and '",
                    multi_robot,
                    "'.lower() == 'true'",
                ]
            )
        ),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_odom_2",
        arguments=["0", "-0.80", "0", "0", "0", "0", "world", "amr_2/odom"],
        parameters=[{"use_sim_time": True}],
    )

    # ========================================================================
    # RVIZ2 WITH DUAL-ROBOT SENSOR DASHBOARD
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
            declare_launch_rsp,
            declare_static_world_tf,
            declare_multi_robot,
            rsp_node_1,
            rsp_node_2,
            static_tf_world_to_odom_1_single,
            static_tf_world_to_odom_1_dual,
            static_tf_world_to_odom_2,
            rviz_node,
        ]
    )
