"""
nav.launch.py
=============
Launches mapping/localization and navigation on top of a running simulation.
Run AFTER sim.launch.py.

Modes (set via 'mode' argument):
  mapping      — slam_toolbox async mapping  (no map file needed)
  localization — slam_toolbox localization   (needs saved map)
  nav_only     — assumes a map is already published (e.g. map_server outside)

Usage:
  # Mapping
  ros2 launch my_bot nav.launch.py mode:=mapping

  # Localization from a saved slam_toolbox .posegraph map
  ros2 launch my_bot nav.launch.py mode:=localization map:=/path/to/map.yaml

  # Pure navigation (external map server already running)
  ros2 launch my_bot nav.launch.py mode:=nav_only

Notes:
  • When mode=mapping the nav2_params map_server yaml_filename is left blank;
    slam_toolbox publishes /map directly.
  • When mode=localization AMCL takes over localisation; slam_toolbox is used
    in localization mode for better scan-matching.
  • Nav2's cmd_vel output is remapped to /cmd_vel_nav so twist_mux can
    arbitrate it against teleop.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():

    pkg_my_bot  = get_package_share_directory("my_bot")
    pkg_nav2    = get_package_share_directory("nav2_bringup")
    pkg_slam    = get_package_share_directory("slam_toolbox")

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="mapping",
        description="One of: mapping | localization | nav_only",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Path to map YAML file (used in localization/nav_only modes)",
    )
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            pkg_my_bot, "config", "mapper_params_online_async.yaml"
        ),
        description="Full path to slam_toolbox params YAML",
    )
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(pkg_my_bot, "config", "nav2_params.yaml"),
        description="Full path to Nav2 params YAML",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2 with nav config?"
    )

    # ── Conditions ────────────────────────────────────────────────────────────
    # PythonExpression must return the lowercase strings 'true'/'false' that
    # IfCondition understands — not Python True/False booleans.
    # is_mapping      = PythonExpression(["'true' if '", LaunchConfiguration("mode"), "' == 'mapping'      else 'false'"])
    # is_localization = PythonExpression(["'true' if '", LaunchConfiguration("mode"), "' == 'localization' else 'false'"])
    # is_not_nav_only = PythonExpression(["'true' if '", LaunchConfiguration("mode"), "' != 'nav_only'     else 'false'"])

    # ── slam_toolbox: async mapping mode ─────────────────────────────────────
    slam_mapping_node = Node(
        #condition=IfCondition(is_mapping),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": True},
        ],
    )

    # ── slam_toolbox: localization mode ──────────────────────────────────────
    slam_localization_params = os.path.join(
        pkg_my_bot, "config", "mapper_params_localization.yaml"
    )
    slam_localization_node = Node(
        # condition=IfCondition(is_localization),
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_localization_params,
            {"use_sim_time": True},
        ],
    )

    # ── Nav2 bringup ─────────────────────────────────────────────────────────
    # We use nav2_bringup's bringup_launch.py but override:
    #   • use_sim_time   = true
    #   • slam           = false  (slam_toolbox node launched separately above)
    #   • localization   = false  (slam_toolbox also handles /map in mapping mode)
    #   • map            = LaunchConfiguration("map")   (for nav_only / localization)
    # Nav2 cmd_vel is remapped from /cmd_vel to /cmd_vel_nav so twist_mux picks it up.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time":   "true",
            "params_file":    LaunchConfiguration("nav2_params_file"),
            "map":            LaunchConfiguration("map"),
            # Disable nav2's internal slam/amcl — we run them separately
            #"slam":           "false",
            "autostart":      "true",
        }.items(),
    )

    # Remap Nav2 cmd_vel output to /cmd_vel_nav for twist_mux
    # nav2_bringup internally uses /cmd_vel; we need to bridge it.
    # The cleanest way: a static relay node that republishes.
    # (Alternatively set cmd_vel_topic in nav2_params; already done via
    #  collision_monitor cmd_vel_out_topic → /cmd_vel_nav in nav2_params.yaml)
    #
    # If your Nav2 version doesn't support per-topic remapping in bringup,
    # uncomment the relay node below:
    #
    # nav2_relay = Node(
    #     package="topic_tools",
    #     executable="relay",
    #     name="nav2_cmd_vel_relay",
    #     arguments=["/cmd_vel", "/cmd_vel_nav"],
    #     parameters=[{"use_sim_time": True}],
    #     output="screen",
    # )

    # ── RViz2 ────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_my_bot, "config", "view_bot.rviz")
    rviz_node = Node(
        # condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ── Assemble ──────────────────────────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        declare_mode,
        declare_map,
        declare_slam_params,
        declare_nav2_params,
        declare_use_rviz,

        # SLAM nodes (conditional)
        # slam_mapping_node,
        slam_localization_node,

        # Nav2
        nav2_bringup,

        # RViz2
        rviz_node,
    ])