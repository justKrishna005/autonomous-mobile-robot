"""
mapping_lidar_only.launch.py
=============================
Lidar-only SLAM mapping on REAL hardware — no wheel odometry, no Gazebo.

This launch file assumes:
  • Your lidar driver (e.g. rplidar_ros) is started SEPARATELY, and is
    already publishing /scan.
  • There is no odom source. slam_toolbox will compute its own odom
    purely from scan matching (mapper_params_lidar_only.yaml is tuned
    for this).
  • There's no robot_state_publisher / joint_state_broadcaster running
    here — this is a minimal "just see the map" setup, not the full
    sim/nav stack.

What this launches:
  1. A static transform publisher: base_link → laser_frame
     (since there's no URDF/robot_state_publisher in this minimal setup,
     we publish this one fixed transform by hand so slam_toolbox knows
     where the lidar sits relative to the robot body)
  2. slam_toolbox (async mapping mode, lidar-only tuned params)
  3. RViz2, pre-configured to show /map + /scan

Usage:
  # 1. Start your lidar driver separately (see commands below), THEN:
  ros2 launch my_bot mapping_lidar_only.launch.py

  # Optional: override the static transform if your lidar isn't mounted
  # at the same offset as the sim robot (default: 0 0 0.14, no rotation)
  ros2 launch my_bot mapping_lidar_only.launch.py laser_x:=0.0 laser_y:=0.0 laser_z:=0.14

Starting the rplidar driver separately (run BEFORE or AFTER this launch
file — order doesn't matter, slam_toolbox will just wait for /scan):

  # Find the device first
  ls /dev/ttyUSB*

  # Give yourself permission if needed
  sudo chmod 666 /dev/ttyUSB0

  # Launch the rplidar driver (A1/A2 - adjust for your model)
  ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser_frame

  # OR, if you only have the node and not the launch file:
  ros2 run rplidar_ros rplidar_node --ros-args \\
      -p serial_port:=/dev/ttyUSB0 \\
      -p frame_id:=laser_frame \\
      -p angle_compensate:=true

Saving the map once you're done driving the robot around:
  ros2 run nav2_map_server map_saver_cli -f ~/my_map
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: '/home/<you>/my_map'"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_my_bot = get_package_share_directory("my_bot")

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_laser_x = DeclareLaunchArgument(
        "laser_x", default_value="0.0",
        description="Lidar X offset from base_link (metres)"
    )
    declare_laser_y = DeclareLaunchArgument(
        "laser_y", default_value="0.0",
        description="Lidar Y offset from base_link (metres)"
    )
    declare_laser_z = DeclareLaunchArgument(
        "laser_z", default_value="0.14",
        description="Lidar Z offset from base_link (metres)"
    )
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            pkg_my_bot, "config", "mapper_params_no_wheel_odom.yaml"
        ),
        description="Full path to the lidar-only slam_toolbox params YAML",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2?"
    )

    # ── Static TF: odom → base_link ──────────────────────────────────────
    # slam_toolbox internally requires odom_frame -> base_frame to exist in
    # TF before it will start publishing map -> odom, even when running
    # purely off scan matching with no real odometry. Since we have no
    # wheel encoders, we publish a zero-offset static transform here so
    # that requirement is satisfied. slam_toolbox then publishes map->odom
    # on top of this, and that's the transform that actually carries the
    # scan-matched pose corrections.
    static_tf_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base_link_tf",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "odom", "base_link",
        ],
        output="screen",
    )

    # ── Static TF: base_link → laser_frame ──────────────────────────────
    # Since there's no robot_state_publisher here, we publish this single
    # fixed transform manually so slam_toolbox's base_frame (base_link)
    # and the lidar's frame (laser_frame, matching your rplidar frame_id)
    # are connected in TF.
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            LaunchConfiguration("laser_x"),
            LaunchConfiguration("laser_y"),
            LaunchConfiguration("laser_z"),
            "0", "0", "0",          # roll, pitch, yaw
            "base_link", "laser_frame",
        ],
        output="screen",
    )

    # ── slam_toolbox: async mapping, lidar-only tuned ───────────────────
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": False},   # REAL hardware — do not use sim time
        ],
    )

    # ── RViz2 ────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_my_bot, "config", "view_bot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    return LaunchDescription([
        declare_laser_x,
        declare_laser_y,
        declare_laser_z,
        declare_slam_params,
        declare_use_rviz,

        static_tf_odom,
        static_tf_laser,
        slam_node,
        rviz_node,
    ])