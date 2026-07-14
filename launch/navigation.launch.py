import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    # Pass your saved map file at launch time:
    #  ros2 launch my_bot navigation.launch.py map:=/home/user/my_map.yaml
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'my_map.yaml'),
        description='Full path to the saved map yaml file')

    pkg_share       = get_package_share_directory('my_bot')
    urdf_path       = os.path.join(pkg_share, 'description',   'my_bot.urdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'diffdrive_controllers.yaml')
    twist_mux_yaml  = os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')
    nav2_params_yaml = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    robot_description_content = ParameterValue(
        Command(['cat ', urdf_path]), value_type=str)

    # RSP
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': use_sim_time}]
    )

    # Control layer
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

    # twist_mux: teleop (priority 100) overrides Nav2 (priority 10)
    # Nav2 controller_server publishes /cmd_vel, remapped to /cmd_vel_nav below
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # RPLidar
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

    # rf2o: handles short-term odom->base_link motion estimation.
    # AMCL handles long-term map->odom drift correction on top of this.
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

    # map_server: loads the saved map from disk
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_yaml,
                    {'yaml_filename': map_file},
                    {'use_sim_time': use_sim_time}]
    )

    # AMCL: localizes robot in saved map, publishes map->odom TF correction
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_yaml, {'use_sim_time': use_sim_time}]
    )

    # lifecycle manager for localization stack
    localization_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # Nav2 planner
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_yaml, {'use_sim_time': use_sim_time}]
    )

    # Nav2 controller: publishes /cmd_vel, remapped to /cmd_vel_nav
    # so twist_mux can route it at lower priority than teleop
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )

    # Nav2 behavior server (spin, back_up, wait)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_yaml, {'use_sim_time': use_sim_time}]
    )

    # Nav2 bt_navigator: executes behavior tree for navigate_to_pose goals
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_yaml, {'use_sim_time': use_sim_time}]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_yaml, {'use_sim_time': use_sim_time}]
    )

    # lifecycle manager for navigation stack
    navigation_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': [
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                    ]}]
    )
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
                'camera_model': 'zed2i',
            'base_frame': 'base_link',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items()
    )
    depth_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
            ('scan', '/depth_scan'),
        ],
        parameters=[os.path.join(pkg_share, 'config',
                                'pointcloud_to_laserscan.yaml')],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        rsp_node,
        controller_manager_node,
        diff_drive_spawner,
        twist_mux_node,
        rplidar_node,
        rf2o_node,
        map_server_node,
        amcl_node,
        localization_lifecycle_manager,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        navigation_lifecycle_manager,
        zed_launch,
        depth_to_scan_node,
    ])
