"""IMU no PC: filtros + TF da câmera. As amostras vêm do edrom_buttond.py (UDP)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('imu_ros_arduino'), 'config', 'imu_filters.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('filter', default_value='ekf',
                              description='mahony ou ekf: alimenta o TF em /tf e os tópicos antigos'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(
            package='imu_ros_arduino',
            executable='imu_filters',
            name='imu_filters',
            output='screen',
            parameters=[config, {
                'filter': LaunchConfiguration('filter'),
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
            }],
        ),
    ])
