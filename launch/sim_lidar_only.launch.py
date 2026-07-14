"""
sim_lidar_only.launch.py
=========================
Validates LIDAR-ONLY SLAM (no odometry) in Gazebo, as a sim rehearsal
before attempting the same on the real crawler with no wheel encoders.

This launch file is DELIBERATELY missing everything encoder-related:
  • NO ros2_control, NO controller_manager
  • NO joint_state_broadcaster → NO /joint_states published
  • NO diff_drive_controller → NO odometry published by a controller
  • Robot motion comes ONLY from Gazebo's built-in DiffDrive system
    plugin (embedded in robot_no_encoder.urdf.xacro), which moves the
    robot in physics from /cmd_vel but publishes no odom/TF of its own.

What this launches:
  1. robot_state_publisher (publishes static URDF-derived TF only —
     base_footprint→base_link→wheels/laser_frame; NOT odom→base_footprint,
     since that's not a fixed joint and there's no controller to publish it)
  2. Gazebo Ignition Fortress
  3. Spawn the no-encoder robot
  4. ros_gz_bridge for /scan and /clock
  5. A static odom → base_footprint transform (zero offset) — required
     so slam_toolbox has a valid odom_frame→base_frame TF to build on,
     exactly as in the real-hardware lidar-only setup. Gazebo's DiffDrive
     plugin does NOT provide this on its own in this configuration, so we
     supply it by hand, meaning slam_toolbox is operating purely off scan
     matching, with zero real motion prior — the same condition as a
     truly encoder-less robot.
  6. slam_toolbox (async mapping, lidar-only tuned params)
  7. twist_mux (teleop only — no nav source needed for this rehearsal)
  8. teleop_twist_keyboard
  9. RViz2

Usage:
  ros2 launch my_bot sim_lidar_only.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_my_bot = get_package_share_directory("my_bot")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_my_bot, "worlds", "my_world.sdf"),
        description="Full path to the Gazebo world file",
    )
    declare_x = DeclareLaunchArgument("x", default_value="0.0")
    declare_y = DeclareLaunchArgument("y", default_value="0.0")
    declare_z = DeclareLaunchArgument("z", default_value="0.1")
    declare_yaw = DeclareLaunchArgument("yaw", default_value="0.0")
    declare_use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    declare_use_teleop = DeclareLaunchArgument("use_teleop", default_value="true")
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            pkg_my_bot, "config", "mapper_params_lidar_only.yaml"
        ),
        description="slam_toolbox params tuned for no-odometry operation",
    )

    # ── Robot description (NO-ENCODER xacro) ─────────────────────────────
    xacro_file = os.path.join(pkg_my_bot, "description", "robot_no_encoder.urdf.xacro")
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # ── robot_state_publisher ─────────────────────────────────────────────
    # Publishes only the URDF's fixed/continuous joint transforms
    # (base_footprint→base_link→wheels, base_link→laser_frame). It does
    # NOT and cannot publish odom→base_footprint — that would normally
    # come from diff_drive_controller, which is absent here by design.
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # ── Gazebo ─────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", LaunchConfiguration("world")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── Spawn robot ────────────────────────────────────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",  "my_bot_no_encoder",
            "-topic", "/robot_description",
            "-x",     LaunchConfiguration("x"),
            "-y",     LaunchConfiguration("y"),
            "-z",     LaunchConfiguration("z"),
            "-Y",     LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    # ── ros_gz_bridge ──────────────────────────────────────────────────────
    # Only /scan and /clock — deliberately NOT bridging any odom topic,
    # since none is published in this configuration.
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # cmd_vel: ROS -> Gazebo, so teleop/twist_mux output reaches
            # the DiffDrive plugin inside the simulated robot.
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        output="screen",
    )

    # ── Static TF: odom → base_footprint ─────────────────────────────────
    # See module docstring — this is the no-odometry stand-in, identical
    # in spirit to the real-hardware lidar-only launch file. Without this,
    # slam_toolbox will never publish map->odom (it silently waits on
    # odom_frame->base_frame existing first).
    static_tf_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base_footprint_tf",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
        output="screen",
    )

    # ── slam_toolbox ───────────────────────────────────────────────────────
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": True},
        ],
    )

    # ── twist_mux (teleop only for this rehearsal) ────────────────────────
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(pkg_my_bot, "config", "twist_mux.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
        output="screen",
    )

    # ── teleop ─────────────────────────────────────────────────────────────
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        remappings=[("/cmd_vel", "/cmd_vel_teleop")],
        # Lowered default speed/turn step to match the robot's reduced
        # DiffDrive velocity caps (0.15 m/s / 0.4 rad/s) — without this,
        # every keypress jumps straight to the hard-capped max, giving
        # no fine control over slow, careful movement.
        parameters=[{"speed": 0.1, "turn": 0.3}],
        prefix="xterm -e",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_teleop")),
    )

    # ── RViz2 ──────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_my_bot, "config", "view_bot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        declare_world,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_use_rviz,
        declare_use_teleop,
        declare_slam_params,

        rsp_node,
        gazebo,
        spawn_robot,
        bridge_node,

        static_tf_odom,
        slam_node,

        twist_mux_node,
        teleop_node,

        rviz_node,
    ])
