import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='edrom_imu_driver',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baudrate': 115200,
                'frame_id': 'imu_link'
            }]
        ),

        # 2. Filtro Madgwick (Funde dados brutos em Orientação)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': 'base_link',
                'gain': 0.1
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/data')
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )
    ])