import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='false here -- real lidar hardware')

    pkg_share = get_package_share_directory('my_bot')
    slam_params_yaml = os.path.join(pkg_share, 'config', 'mapper_params_no_wheel_odom.yaml')

    # RPLidar driver -- publishes /scan
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',   # CHANGE to match your actual port
            'serial_baudrate': 115200,
            'inverted': False,
            'angle_compensate': True,
            'frame_id': 'laser_frame',
            'use_sim_time': use_sim_time,
        }]
    )


    # No robot_state_publisher in this minimal stack, so nothing else
    # publishes base_link -> laser_frame. Without this, rf2o has no tf
    # path to its base_frame_id and silently can't produce odometry --
    # this replaces the manual `ros2 run tf2_ros static_transform_publisher`
    # step you were running by hand.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # Pure-lidar odometry -- replaces wheel/encoder odom
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

    # slam_toolbox -- builds the map live
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_yaml, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rplidar_node,
        static_tf_node,
        rf2o_node,
        slam_toolbox_node,
    ])