#!/usr/bin/env python3
# coding=utf-8
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    args = [
        DeclareLaunchArgument("imu_connected",  default_value="false"),
        DeclareLaunchArgument("imu_port",       default_value="/dev/ttyIMU"),
        DeclareLaunchArgument("simulation",     default_value="false"),
        DeclareLaunchArgument("tick_hz",        default_value="20.0"),
        DeclareLaunchArgument("use_getup_srv",  default_value="false"),
    ]

    # Resolvido em lazy substitution — não falha se bhv_simulator não estiver instalado
    sim_launch_path = PathJoinSubstitution([
        FindPackageShare("bhv_simulator"),
        "launch", "behaviour_simulator.launch.py",
    ])

    return LaunchDescription([
        *args,

        # Simulação (opcional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
            condition=IfCondition(LaunchConfiguration("simulation")),
        ),

        # Nó principal — DSD
        Node(
            package="transitions_and_states",
            executable="dsd_node",
            name="dsd_behavior",
            output="screen",
            parameters=[{
                "tick_hz":       LaunchConfiguration("tick_hz"),
                "use_getup_srv": LaunchConfiguration("use_getup_srv"),
            }],
        ),

        # ROSPacker — mantido sem alteração
        Node(
            package="sensor_observer",
            executable="ros_packer",
            name="ros_packer",
            output="screen",
        ),

        Node(
            package="states_routine",
            executable="kicking_routine",
            name="kicking_routine",
            output="screen",
        ),

        Node(
            package="states_routine",
            executable="walking_routine",
            name="walking_routine",
            output="screen",
        ),

        Node(
            package="states_routine",
            executable="searching_routine",
            name="searching_routine",
            output="screen",
        ),

        Node(
            package="states_routine",
            executable="aligning_body_routine",
            name="aligning_body_routine",
            output="screen",
        ),

        Node(
            package="states_routine",
            executable="getting_up_routine",
            name="getting_up_routine",
            output="screen",
        ),

        Node(
            package="states_routine",
            executable="idle_march_routine",
            name="idle_march_routine",
            output="screen",
        ),

        Node(
            package="states_routine",
            executable="idle_routine",
            name="idle_routine",
            output="screen",
        ),
        
        # IMU (opcional)
        Node(
            package="imu_ros_arduino",
            executable="imu_read",
            name="imu_ros_arduino",
            output="screen",
            condition=IfCondition(LaunchConfiguration("imu_connected")),
            parameters=[{"port": LaunchConfiguration("imu_port")}],
        ),
    ])