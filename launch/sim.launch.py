"""
sim.launch.py
=============
Launches the full simulation stack:
  • Gazebo Ignition Fortress (gz sim)
  • robot_state_publisher  (publishes /robot_description + static TF)
  • ros_gz_bridge          (Gazebo ↔ ROS 2 topic bridging)
  • Spawn the robot in Gazebo
  • gz_ros2_control        (loaded from inside URDF via Gazebo plugin)
  • controller_manager spawners: joint_state_broadcaster, diff_drive_controller
  • twist_mux
  • teleop_twist_keyboard  (publishes to /cmd_vel_teleop)
  • RViz2

Usage:
  ros2 launch my_bot sim.launch.py
  ros2 launch my_bot sim.launch.py world:=<path/to/world.sdf> x:=1.0 y:=0.0
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_my_bot   = get_package_share_directory("my_bot")
    pkg_ros_gz   = get_package_share_directory("ros_gz_sim")

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_my_bot, "worlds", "my_world.sdf"),
        description="Full path to the Gazebo world file",
    )
    declare_x = DeclareLaunchArgument("x", default_value="0.0",
                                      description="Robot spawn X position")
    declare_y = DeclareLaunchArgument("y", default_value="0.0",
                                      description="Robot spawn Y position")
    declare_z = DeclareLaunchArgument("z", default_value="0.1",
                                      description="Robot spawn Z position")
    declare_yaw = DeclareLaunchArgument("yaw", default_value="0.0",
                                        description="Robot spawn yaw")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2?"
    )
    declare_use_teleop = DeclareLaunchArgument(
        "use_teleop", default_value="true",
        description="Launch teleop_twist_keyboard?"
    )

    # ── Robot description (xacro → URDF string) ───────────────────────────────
    xacro_file = os.path.join(pkg_my_bot, "description", "robot.urdf.xacro")
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file]
    )
    robot_description = {"robot_description": robot_description_content}

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # 1. robot_state_publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # 2. Gazebo Ignition Fortress
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", LaunchConfiguration("world")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # 3. Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",  "my_bot",
            "-topic", "/robot_description",
            "-x",     LaunchConfiguration("x"),
            "-y",     LaunchConfiguration("y"),
            "-z",     LaunchConfiguration("z"),
            "-Y",     LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    # 4. ros_gz_bridge — bridges Gazebo topics to ROS 2
    #    LaserScan:   /scan          (gz → ros)
    #    Clock:       /clock         (gz → ros)
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # 5. Controller spawners (activated after spawn so controller_manager is up)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn diff_drive after joint_state_broadcaster is active
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # Start controller spawners only after robot is spawned
    delayed_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # 6. twist_mux
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(pkg_my_bot, "config", "twist_mux.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[
            # twist_mux output → diff_drive_controller input
            ("/cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped"),
        ],
        output="screen",
    )

    # 7. teleop_twist_keyboard  → /cmd_vel_teleop
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        remappings=[("/cmd_vel", "/cmd_vel_teleop")],
        prefix="xterm -e",          # opens in its own terminal window
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_teleop")),
    )

    # 8. RViz2
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

    # ── Assemble ──────────────────────────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        declare_world,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_use_rviz,
        declare_use_teleop,

        # Nodes
        rsp_node,
        gazebo,
        bridge_node,
        spawn_robot,

        # Controllers (event-driven ordering)
        delayed_jsb_spawner,
        delayed_diff_drive_spawner,

        # Mux + teleop
        twist_mux_node,
        teleop_node,

        # Visualisation
        rviz_node,
    ])
