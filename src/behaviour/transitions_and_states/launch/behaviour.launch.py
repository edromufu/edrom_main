#!/usr/bin/env python3
# coding=utf-8
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Serviço systemd que lê a serial do ESP32 (edrom_buttond.py) e repassa as
# amostras das IMUs por UDP para os nós imu_filters. Não é lançado aqui.
BUTTONS_SERVICE = 'edrom-buttons'


def check_buttons_service(context):
    """Avisa na subida se o edrom-buttons não estiver rodando (sem ele não chega IMU)."""
    if LaunchConfiguration('imu_connected').perform(context).lower() not in ('true', '1'):
        return []
    try:
        state = subprocess.run(['systemctl', 'is-active', BUTTONS_SERVICE],
                               capture_output=True, text=True, timeout=2.0).stdout.strip()
    except (OSError, subprocess.SubprocessError) as e:
        state = f'desconhecido ({e})'
    if state == 'active':
        return [LogInfo(msg=f'[behaviour] {BUTTONS_SERVICE}: ativo, IMUs chegando por UDP.')]
    return [LogInfo(msg=f'[behaviour] AVISO: serviço {BUTTONS_SERVICE} está "{state}". '
                        'Sem ele as IMUs não recebem amostras (sem detecção de queda nem TF da câmera). '
                        f'Suba com: sudo systemctl start {BUTTONS_SERVICE} '
                        f'(logs: journalctl -u {BUTTONS_SERVICE})')]


def generate_launch_description():
    # Argumentos
    imu_connected_arg = DeclareLaunchArgument(
        'imu_connected', default_value='true',
        description='Sobe os nós das IMUs (imu_filters)')
    imu_head_arg = DeclareLaunchArgument(
        'imu_head', default_value='true',
        description='IMU da cabeça: /imu/head e TF base_footprint -> camera_optical_frame')
    imu_torso_arg = DeclareLaunchArgument(
        'imu_torso', default_value='true',
        description='IMU do tronco: /behaviour/imu_* (queda) e /imu/ekf/rpy, /imu/data_raw (torso)')

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
        imu_head_arg,
        imu_torso_arg,
        routines_launch,

        # Máquina de Estados
        Node(
            package='transitions_and_states',
            executable='state_machine_receiver',
            name='behaviour',
            output='screen'
        ),

        # Interpretação (ros_packer já roda o fall_interpreter, que lê /behaviour/imu_*)
        Node(
            package='sensor_observer',
            executable='ros_packer',
            name='ros_packer',
            output='screen'
        ),

        # IMUs (condicional): dois MPU-6050 no ESP32 dos botões. O
        # edrom_buttond.py (serviço edrom-buttons) é o dono da serial e repassa
        # as amostras por UDP: cabeça (linhas "IMU") -> 5600, tronco ("IMU2") -> 5601.
        # Uma instância do imu_filters por IMU (config/imu_filters.yaml):
        #   imu_head   /imu/head e TF base_footprint -> camera_optical_frame
        #   imu_torso  /behaviour/imu_accel|gyro|roll (fall_interpreter) e
        #              /imu/ekf/rpy, /imu/data_raw (controlador de torso do aurea_walk)
        OpaqueFunction(function=check_buttons_service),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('imu_ros_arduino'),
                'launch',
                'imu_filters.launch.py'
            )),
            launch_arguments={
                'head': LaunchConfiguration('imu_head'),
                'torso': LaunchConfiguration('imu_torso'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('imu_connected')),
        )
    ])
