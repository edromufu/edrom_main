"""Um soccer_ipm por filtro + o impressor. Não sobe câmera nem visão: rode
antes imu_filters.launch.py e vision.launch.py (o soccer_ipm do vision.launch
pode continuar rodando, não atrapalha).

Cada soccer_ipm fica no namespace /<filtro>, lê o TF em /<filtro>/tf e as
mesmas detecções (/*_in_image); publica em /<filtro>/*_relative.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOPICOS = ['balls_in_image', 'goal_posts_in_image', 'robots_in_image']


def nodes(context):
    filtros = LaunchConfiguration('filters').perform(context).split(',')
    out = [Node(package='imu_ros_arduino', executable='ipm_filter_print', output='screen',
                parameters=[{'filters': filtros}])]
    for f in filtros:
        out.append(Node(
            package='soccer_ipm', executable='ipm', name='soccer_ipm_node', namespace=f,
            parameters=[{'output_frame': 'base_footprint', 'balls.ball_diameter': 0.14,
                         'use_distortion': True}],
            remappings=[('/tf', f'/{f}/tf'), ('/tf_static', f'/{f}/tf_static'),
                        ('camera_info', '/camera/camera_info')]
                       + [(t, f'/{t}') for t in TOPICOS]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('filters', default_value='mahony,ekf'),
        OpaqueFunction(function=nodes),
    ])
