#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("factory_delivery_amr_navigation")

    declare_robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="amr_1",
        description="Robot namespace used for SLAM scanning (e.g. amr_1 or amr_2)",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 for real-time SLAM map visualization",
    )

    robot_ns = LaunchConfiguration("robot_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")

    slam_params_template = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    rviz_config_template = os.path.join(pkg_share, "rviz", "navigation.rviz")

    replaced_slam_params = ReplaceString(
        source_file=slam_params_template,
        replacements={
            "<robot_ns>": robot_ns,
            "<map_frame>": "map",
            "<map_name>": "/map",
        },
    )

    configured_slam_params = ParameterFile(
        RewrittenYaml(
            source_file=replaced_slam_params,
            root_key=robot_ns,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    scan_color = PythonExpression(
        ["'255; 80; 0' if '", robot_ns, "' == 'amr_2' else '0; 255; 100'"]
    )

    configured_rviz_config = ReplaceString(
        source_file=rviz_config_template,
        replacements={
            "<robot_namespace>": ("/", robot_ns),
            "<scan_color>": scan_color,
        },
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=robot_ns,
        output="screen",
        parameters=[configured_slam_params],
        remappings=[
            ("map", "/map"),
            ("map_metadata", "/map_metadata"),
        ],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        namespace=robot_ns,
        output="screen",
        arguments=["-d", configured_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/map", "/map"),
            ("/goal_pose", "goal_pose"),
            ("/clicked_point", "clicked_point"),
            ("/initialpose", "initialpose"),
        ],
    )

    return LaunchDescription(
        [
            declare_robot_ns,
            declare_use_sim_time,
            declare_rviz,
            slam_node,
            rviz_node,
        ]
    )
