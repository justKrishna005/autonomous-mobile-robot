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
        'use_sim_time', default_value='false')

    pkg_share = get_package_share_directory('my_bot')
    urdf_path         = os.path.join(pkg_share, 'description',   'my_bot.urdf')
    controllers_yaml  = os.path.join(pkg_share, 'config', 'diffdrive_controllers.yaml')
    twist_mux_yaml    = os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')
    slam_params_yaml  = os.path.join(pkg_share, 'config', 'mapper_params_no_wheel_odom.yaml')

    robot_description_content = ParameterValue(
        Command(['cat ', urdf_path]), value_type=str)

    # RSP -- publishes full TF tree from URDF (base_link, wheels, laser_frame)
    # No separate static_transform_publisher needed anymore.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': use_sim_time}]
    )

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

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # RPLidar -- USB0, lidar is always first USB device plugged in
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': use_sim_time,
        }]
    )

    # rf2o -- pure lidar odometry, publishes odom->base_link TF + /odom topic
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
            'use_sim_time': use_sim_time,
        }]
    )

    # slam_toolbox -- async mapping, publishes map->odom TF + /map topic
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_yaml, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp_node,
        controller_manager_node,
        diff_drive_spawner,
        twist_mux_node,
        rplidar_node,
        rf2o_node,
        slam_toolbox_node,
    ])
