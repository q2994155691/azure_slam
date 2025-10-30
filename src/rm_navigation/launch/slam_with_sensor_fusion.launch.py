from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 配置文件路徑
    nav_dir = get_package_share_directory('rm_navigation')
    ekf_config = os.path.join(nav_dir, 'config', 'ekf_localization.yaml')
    slam_config = os.path.join(nav_dir, 'params', 'mapper_params_online_async.yaml')
    
    # 聲明 launch 參數
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    
    return LaunchDescription([
        # 1. 啟動 RPLIDAR C1
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': scan_mode,
            }],
            output='screen'
        ),
        
        # 2. 啟動 IMU
        Node(
            package='dm_imu_ros2',
            executable='dm_imu_node',
            name='dm_imu_node',
            output='screen'
        ),
        
        # 3. 啟動 RF2O 激光里程計
        Node(
	    package='rf2o_laser_odometry',
	    executable='rf2o_laser_odometry_node',
	    name='rf2o_laser_odometry',
	    output='screen',
	    parameters=[{
		'laser_scan_topic': '/scan',
		'odom_topic': '/odom_rf2o',
		'publish_tf': False,
		'base_frame_id': 'base_footprint',
		'odom_frame_id': 'odom',
		'freq': 20.0,  # 從 10 提高到 20 Hz - 更頻繁的里程計更新
		'verbose': False,
		'max_keyframes': 50,  # 提高關鍵幀數量
	    }]
	),
        
        # 4. 啟動 robot_localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odometry/local')]
        ),
        
        # 5. 啟動 SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
        ),
        
        # 6. 靜態 TF (base_footprint 到 base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        
        # 7. 靜態 TF (base_link 到 laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # 8. 靜態 TF (base_link 到 imu_link) - IMU 相對於 base_link 的位置
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        ),
    ])

