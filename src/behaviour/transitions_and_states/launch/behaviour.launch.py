#!/usr/bin/env python3
# coding=utf-8    

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition   
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Argumentos
    imu_connected_arg = DeclareLaunchArgument(
        "imu_connected",
        default_value="false"
    )
    imu_port_arg = DeclareLaunchArgument(
        "imu_port",
        default_value="/dev/ttyIMU"
    )
    simulation_arg = DeclareLaunchArgument(
        "simulation",
        default_value="true"
    )

    # Configurações
    imu_connected = LaunchConfiguration("imu_connected")
    imu_port = LaunchConfiguration("imu_port")
    simulation = LaunchConfiguration("simulation")

    # Caminho do outro launch
    behaviour_sim_launch = os.path.join(
        get_package_share_directory('bhv_simulator'),
        'launch',
        'behaviour_simulator.launch.py'
    )

    return LaunchDescription([
        # Declaração de argumentos
        imu_connected_arg,
        imu_port_arg,
        simulation_arg,

        # Inclui outro launch (condicional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(behaviour_sim_launch),
            condition=IfCondition(simulation)
        ),

        # Máquina de Estados
        Node(
            package="transitions_and_states",
            executable="state_machine_receiver",
            name="state_machine",
            output="screen"
        ),

        # Interpretação
        Node(
            package="sensor_observer",
            executable="ros_packer",
            name="ros_packer",
            output="screen"
        ),

        # Simulador (condicional)
        Node(
            package="bhv_simulator",
            executable="bhv_sim",
            name="bhv_simulator",
            output="screen",
            condition=IfCondition(simulation),
        ),

        # Leitura do IMU (condicional)
        Node(
            package="imu_ros_arduino",
            executable="imu_read",
            name="imu_ros_arduino",
            output="screen",
            condition=IfCondition(imu_connected),
            parameters=[{"port": imu_port}]
        )
    ])
