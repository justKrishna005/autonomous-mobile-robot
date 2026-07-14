import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='false here -- real hardware')

    pkg_share = get_package_share_directory('my_bot')
    urdf_path = os.path.join(pkg_share, 'description', 'my_bot.urdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'diffdrive_controllers.yaml')
    twist_mux_yaml = os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')

    robot_description_content = ParameterValue(
        Command(['cat ', urdf_path]), value_type=str)

    # robot_state_publisher -- needed so controller_manager can read the
    # ros2_control block out of the URDF. No lidar/laser_frame relevance
    # here, this is purely for the control layer.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': use_sim_time}]
    )

    # controller_manager -- loads diffdrive_arduino hardware + diff_cont controller
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description_content},
                    controllers_yaml]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager']
    )
    # joint_state_broadcaster intentionally not spawned -- not needed for
    # teleop or for rf2o/slam_toolbox later; purely cosmetic in RViz.

    # twist_mux -- single source for now (teleop), arbitration matters once
    # Nav2 is added later as a second, lower-priority source.
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
        # CHECK this target topic with `ros2 topic list` after bringup --
        # depends on your diff_drive_controller version/use_stamped_vel.
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp_node,
        controller_manager_node,
        diff_drive_spawner,
        twist_mux_node,
    ])