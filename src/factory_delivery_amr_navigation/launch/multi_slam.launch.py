#!/usr/bin/env python3
"""
Multi-Robot Real-Time SLAM Launch (amr_1 + amr_2 + Map Merge -> /map).

Launches two independent async_slam_toolbox_node instances under `/amr_1`
and `/amr_2` with separate map frames (`amr_1/map`, `amr_2/map`) and topics
(`/amr_1/map`, `/amr_2/map`), plus `multi_robot_map_merge.py` to broadcast
static transforms from `map` and publish the unified real-time `/map`.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString
from nav2_common.launch import RewrittenYaml


def _make_slam_node(
    pkg_share: str,
    robot_ns: str,
    use_sim_time: LaunchConfiguration,
) -> Node:
    slam_params_template = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    replaced = ReplaceString(
        source_file=slam_params_template,
        replacements={
            "<robot_ns>": robot_ns,
            "<map_frame>": f"{robot_ns}/map",
            "<map_name>": f"/{robot_ns}/map",
        },
    )
    configured = ParameterFile(
        RewrittenYaml(
            source_file=replaced,
            root_key=robot_ns,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    return Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=robot_ns,
        output="screen",
        parameters=[configured],
        remappings=[
            ("/map", f"/{robot_ns}/map"),
            ("/map_metadata", f"/{robot_ns}/map_metadata"),
        ],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("factory_delivery_amr_navigation")
    gazebo_share = get_package_share_directory("factory_delivery_amr_gazebo")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 showing the merged /map and both robots",
    )
    declare_amr_1_x = DeclareLaunchArgument("amr_1_init_x", default_value="0.0")
    declare_amr_1_y = DeclareLaunchArgument("amr_1_init_y", default_value="0.80")
    declare_amr_1_yaw = DeclareLaunchArgument("amr_1_init_yaw", default_value="0.0")
    declare_amr_2_x = DeclareLaunchArgument("amr_2_init_x", default_value="0.0")
    declare_amr_2_y = DeclareLaunchArgument("amr_2_init_y", default_value="-0.80")
    declare_amr_2_yaw = DeclareLaunchArgument("amr_2_init_yaw", default_value="0.0")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")

    slam_amr_1 = _make_slam_node(pkg_share, "amr_1", use_sim_time)
    slam_amr_2 = _make_slam_node(pkg_share, "amr_2", use_sim_time)

    map_merge_node = Node(
        package="factory_delivery_amr_navigation",
        executable="multi_robot_map_merge.py",
        name="multi_robot_map_merge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "global_frame": "map",
                "merge_rate_hz": 2.0,
                "amr_1_init_x": LaunchConfiguration("amr_1_init_x"),
                "amr_1_init_y": LaunchConfiguration("amr_1_init_y"),
                "amr_1_init_yaw": LaunchConfiguration("amr_1_init_yaw"),
                "amr_2_init_x": LaunchConfiguration("amr_2_init_x"),
                "amr_2_init_y": LaunchConfiguration("amr_2_init_y"),
                "amr_2_init_yaw": LaunchConfiguration("amr_2_init_yaw"),
            }
        ],
    )

    rviz_config_template = os.path.join(pkg_share, "rviz", "navigation.rviz")
    configured_rviz_config = ReplaceString(
        source_file=rviz_config_template,
        replacements={"<robot_namespace>": "/amr_1"},
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_multi_slam",
        output="screen",
        arguments=["-d", configured_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_amr_1_x,
            declare_amr_1_y,
            declare_amr_1_yaw,
            declare_amr_2_x,
            declare_amr_2_y,
            declare_amr_2_yaw,
            slam_amr_1,
            slam_amr_2,
            map_merge_node,
            rviz_node,
        ]
    )
