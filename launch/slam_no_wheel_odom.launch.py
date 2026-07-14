import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true when running in Gazebo, false on the real Jetson bot')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('my_bot'),
            'config', 'mapper_params_no_wheel_odom.yaml'),
        description='Path to the slam_toolbox params file')

    # Pure-lidar odometry. This REPLACES wheel/encoder odom -- it generates
    # the odom_frame -> base_frame transform that slam_toolbox needs as a
    # motion prior, using only /scan. No encoders anywhere in this chain.
    #
    # Build from source (not in apt for most distros):
    #   cd ~/your_ws/src
    #   git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
    #   cd ~/your_ws && rosdep install --from-paths src --ignore-src -r -y
    #   colcon build --packages-select rf2o_laser_odometry
    #   source install/setup.bash
    #
    # Note: rf2o requires a valid static tf from your lidar's frame to
    # base_frame_id to already be publishing (your URDF/robot_state_publisher
    # should be handling this) -- it will not work without it.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'base_link',   # CHANGE to match your URDF
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,   # CHANGE to match your lidar's actual scan rate (Hz)
            'use_sim_time': use_sim_time,
        }]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        rf2o_node,
        #slam_toolbox_node,
    ])