import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav_dir = get_package_share_directory('rm_navigation')
    ekf_config = os.path.join(nav_dir, 'config', 'ekf_imu_only.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true'
        ),
        
        # 1. IMU 驅動
        Node(
            package='dm_imu_ros2',
            executable='dm_imu_node',
            name='dm_imu_node',
            output='screen'
        ),
        
        # 2. EKF 濾波器 (純 IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        
        # 3. 靜態 TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        
        # 4. 靜態 TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        ),
    ])
