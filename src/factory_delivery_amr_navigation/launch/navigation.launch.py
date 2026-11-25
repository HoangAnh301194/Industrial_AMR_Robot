#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("factory_delivery_amr_navigation")
    default_map_file = os.path.join(pkg_share, "maps", "warehouse_map.yaml")
    nav2_params_template = os.path.join(pkg_share, "config", "nav2_params.yaml")
    rviz_config_template = os.path.join(pkg_share, "rviz", "navigation.rviz")

    declare_robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="amr_1",
        description="Target robot namespace (e.g. amr_1 or amr_2)",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map_file,
        description="Full path to the static warehouse map YAML file",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start Nav2 lifecycle nodes",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 Navigation dashboard",
    )
    declare_launch_map_server = DeclareLaunchArgument(
        "launch_map_server",
        default_value="true",
        description="Launch global map_server (set false if launching a second robot)",
    )
    declare_set_initial_pose = DeclareLaunchArgument(
        "set_initial_pose",
        default_value="false",
        description="Auto-initialize AMCL at init_x/init_y if true; false lets user pick start pose in RViz",
    )
    declare_init_x = DeclareLaunchArgument(
        "init_x",
        default_value="0.0",
        description="Initial X pose for AMCL (m)",
    )
    declare_init_y = DeclareLaunchArgument(
        "init_y",
        default_value="0.0",
        description="Initial Y pose for AMCL (m)",
    )
    declare_init_yaw = DeclareLaunchArgument(
        "init_yaw",
        default_value="0.0",
        description="Initial Yaw angle for AMCL (rad)",
    )

    robot_ns = LaunchConfiguration("robot_ns")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("rviz")
    launch_map_server = LaunchConfiguration("launch_map_server")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    init_x = LaunchConfiguration("init_x")
    init_y = LaunchConfiguration("init_y")
    init_yaw = LaunchConfiguration("init_yaw")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    replaced_params = ReplaceString(
        source_file=nav2_params_template,
        replacements={
            "<robot_ns>": robot_ns,
            "<set_initial_pose>": set_initial_pose,
            "<init_x>": init_x,
            "<init_y>": init_y,
            "<init_yaw>": init_yaw,
        },
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=replaced_params,
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
            "<tf_prefix>": robot_ns,
        },
    )

    # Global Map Server (publishes /map shared across robots)
    map_server_group = GroupAction(
        condition=IfCondition(launch_map_server),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaml_filename": map_yaml_file,
                    }
                ],
            ),
            TimerAction(
                period=1.5,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_map",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": autostart,
                                "node_names": ["map_server"],
                            }
                        ],
                    )
                ],
            ),
        ],
    )

    localization_lifecycle_nodes = ["amcl"]

    navigation_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Per-robot Nav2 stack keeping global /tf and /tf_static
    robot_nav_group = GroupAction(
        actions=[
            PushRosNamespace(robot_ns),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[configured_params],
                remappings=[("/map", "/map")],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_localization",
                        namespace=robot_ns,
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": autostart,
                                "node_names": localization_lifecycle_nodes,
                            }
                        ],
                    )
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", "cmd_vel"),
                ],
            ),
            TimerAction(
                period=2.5,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        namespace=robot_ns,
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": autostart,
                                "node_names": navigation_lifecycle_nodes,
                            }
                        ],
                    )
                ],
            ),
        ]
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_navigation",
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
            stdout_linebuf_envvar,
            declare_robot_ns,
            declare_map,
            declare_use_sim_time,
            declare_autostart,
            declare_rviz,
            declare_launch_map_server,
            declare_set_initial_pose,
            declare_init_x,
            declare_init_y,
            declare_init_yaw,
            map_server_group,
            robot_nav_group,
            rviz_node,
        ]
    )
