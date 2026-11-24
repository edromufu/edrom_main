#!/usr/bin/env python3
# coding=utf-8    
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition   
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Argumentos
    imu_connected_arg = DeclareLaunchArgument(
        "imu_connected",
        default_value="true"
    )
    imu_port_arg = DeclareLaunchArgument(
        "imu_port",
        default_value="/dev/ttyIMU"
    )

    # Configurações
    imu_connected = LaunchConfiguration("imu_connected")
    imu_port = LaunchConfiguration("imu_port")

    routines_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('states_routine'),
            'launch',
            'routines.launch.py'
        ))
    )
    return LaunchDescription([
        # Declaração de argumentos
        imu_connected_arg,
        imu_port_arg,
        routines_launch,

        # Máquina de Estados
        Node(
            package="transitions_and_states",
            executable="state_machine_receiver",
            name="behaviour",
            output="screen"
        ),

        # Interpretação
        Node(
            package="sensor_observer",
            executable="ros_packer",
            name="ros_packer",
            output="screen"
        ),

        # IMU (condicional): o MPU-6050 está no ESP32 dos botões; o
        # edrom_buttond.py repassa as amostras por UDP e o imu_filters filtra
        # no PC e publica /behaviour/imu_* e o TF da câmera.
        # (imu_port não é mais usado: a serial é do edrom_buttond.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('imu_ros_arduino'),
                'launch',
                'imu_filters.launch.py'
            )),
            condition=IfCondition(imu_connected),
        )
    ])
