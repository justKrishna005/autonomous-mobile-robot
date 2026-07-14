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
        description='Always false here -- this is the real-hardware bringup')

    pkg_share = get_package_share_directory('my_bot')
    urdf_path = os.path.join(pkg_share, 'description', 'my_bot.urdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'diffdrive_controllers.yaml')
    twist_mux_yaml = os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')
    slam_params_yaml = os.path.join(pkg_share, 'config', 'mapper_params_no_wheel_odom.yaml')

    robot_description_content = ParameterValue(
        Command(['cat ', urdf_path]), value_type=str)

    # --- robot_state_publisher: reads URDF, broadcasts base_link -> wheel/laser TFs ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': use_sim_time}]
    )

    # --- controller_manager: loads diffdrive_arduino hardware + diff_cont controller ---
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
    # NOTE: joint_state_broadcaster intentionally NOT spawned -- nothing in
    # this chain (rf2o / slam_toolbox) reads /joint_states. Only cosmetic
    # effect of skipping it: wheel links won't visually spin in RViz.

    # --- RPLidar driver: needed for rf2o + slam_toolbox to have /scan at all ---
    # ADDED -- wasn't in your listed node set but nothing downstream works without it.
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',   # CHANGE to match your actual port
            'frame_id': 'laser_frame',
            'use_sim_time': use_sim_time,
        }]
    )

    # --- rf2o: pure-lidar odometry, replaces wheel/encoder odom ---
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,   # CHANGE to match your lidar's actual scan rate (Hz)
            'use_sim_time': use_sim_time,
        }]
    )

    # --- slam_toolbox: builds the map live as you teleop around ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_yaml, {'use_sim_time': use_sim_time}]
    )

    # --- twist_mux: arbitrates cmd_vel sources, outputs to what diff_cont expects ---
    # Output topic "cmd_vel_out" is twist_mux's fixed publisher topic name;
    # remapped here directly to diff_cont's input. Confirm the exact target
    # topic with `ros2 topic list` after controller_manager is up -- it may
    # be cmd_vel_unstamped or cmd_vel depending on your diff_drive_controller
    # version/use_stamped_vel setting.
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp_node,
        controller_manager_node,
        diff_drive_spawner,
        rplidar_node,
        rf2o_node,
        slam_toolbox_node,
        twist_mux_node,
    ])
