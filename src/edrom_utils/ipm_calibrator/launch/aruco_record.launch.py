"""Inicia a câmera (mesma config do vision.launch) e o aruco_record.

Varredura automática da cabeça (robô na first pose, modo POSE do botão 2):
    ros2 launch ipm_calibrator aruco_record.launch.py varredura:=True imu_topic:=/imu/data_raw
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    # Mesmo usb_cam.yaml do vision.launch e do camera_calibrator
    usb_cam_params = os.path.join(
        get_package_share_directory('object_finder'), 'config', 'usb_cam.yaml')
    with open(usb_cam_params) as f:
        camera_name = next(iter(yaml.safe_load(f).values()))['ros__parameters']['camera_name']

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_rotation',
            default_value='180',
            description='Rotação (graus, sentido horário: 0, 90, 180 ou 270) da janela, '
                        'para compensar a câmera montada girada (mesmo do vision.launch)',
        ),
        DeclareLaunchArgument(
            'varredura', default_value='False',
            description='Comanda a cabeça por uma grade pan x tilt e grava em cada pose'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data_raw',
            description='Tópico do IMU da cabeça (sensor_msgs/Imu)'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera',
            output='screen',
            parameters=[usb_cam_params, {
                'camera_info_url':
                    f'package://object_finder/config/{camera_name}_camera-info.yaml',
            }],
        ),
        Node(
            package='ipm_calibrator',
            executable='aruco_record',
            output='screen',
            parameters=[{
                'image_rotation': ParameterValue(
                    LaunchConfiguration('image_rotation'), value_type=int),
                'varredura': ParameterValue(LaunchConfiguration('varredura'), value_type=bool),
                'imu_topic': LaunchConfiguration('imu_topic'),
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),
    ])
