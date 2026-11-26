"""IMUs da cabeça e do tronco (EKF). As amostras vêm do edrom_buttond.py por UDP."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('imu_ros_arduino'), 'config', 'imu_filters.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('head', default_value='true', description='IMU da cabeça (TF da câmera)'),
        DeclareLaunchArgument('torso', default_value='true', description='IMU do tronco (queda)'),
        Node(package='imu_ros_arduino', executable='imu_filters', name='imu_head', output='screen',
             parameters=[config], condition=IfCondition(LaunchConfiguration('head'))),
        Node(package='imu_ros_arduino', executable='imu_filters', name='imu_torso', output='screen',
             parameters=[config], condition=IfCondition(LaunchConfiguration('torso'))),
    ])
