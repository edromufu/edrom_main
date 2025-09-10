import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('aurea_walk')
    
    config_file = os.path.join(pkg_dir, 'config', 'walking_params.yaml')
    config = os.path.join(
        get_package_share_directory('aurea_kick'),
        'config',
        'kick_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='aurea_walk',
            executable='walking_engine_node',
            name='walking_engine_node',
            parameters=[config_file, 
            {'backlash_offset_hp': -0.2}, # Comece com um valor pequeno
            {'servo_kp_gain': 3.0} ],# Valor de exemplo, sintonize para seu motor],
            output='screen'
        ),
        Node(
            package='aurea_walk',
            executable='ik_node',
            name='smart_ik_node'
        ),

        Node(
            package='aurea_kick',
            executable='kick_node',
            name='kick_node',
            parameters=[config],
            output='screen'
        )

    ])
