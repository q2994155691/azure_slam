import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('rm_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # EKF 配置文件路徑
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf_localization.yaml')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true')
    
    # 1. RPLidar C1 驅動
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )
    
    # 2. IMU 驅動
    imu_node = Node(
        package='dm_imu_ros2',
        executable='dm_imu_node',
        name='dm_imu_node',
        output='screen'
    )
    
    # 3. RF2O 雷達里程計（不發布 TF，由 robot_localization 發布）
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,  # 關鍵：不發布 TF
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }]
    )
    
    # 4. robot_localization (EKF 傳感器融合)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odometry/local')]
    )
    
    # 5. TF: base_footprint -> base_link
    base_footprint_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'base_link']
    )
    
    # 6. TF: base_link -> laser
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0.1',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'laser']
    )
    
    # 7. TF: base_link -> imu_link
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['--x', '0', '--y', '0', '--z', '0.05',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'imu_link']
    )
    
    # 8. SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 9. Map Saver
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(rplidar_launch)
    ld.add_action(imu_node)           # 新增：IMU
    ld.add_action(rf2o_node)
    ld.add_action(ekf_node)            # 新增：EKF 融合
    ld.add_action(base_footprint_to_link)
    ld.add_action(base_to_laser)
    ld.add_action(base_to_imu)         # 新增：IMU TF
    ld.add_action(slam_toolbox)
    ld.add_action(map_saver)
    
    return ld
