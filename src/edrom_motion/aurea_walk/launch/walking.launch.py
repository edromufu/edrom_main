import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('aurea_walk')
    
    #config_file = os.path.join(pkg_dir, 'configs', 'walking_params.yaml')
    #config = os.path.join(
    #    get_package_share_directory('aurea_kick'),
    #    'configs',
    #    'kick_params.yaml'
    #)

    return LaunchDescription([
        Node(
            package='aurea_walk',
            executable='walking_engine_node',
            name='walking_engine_node',
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
            output='screen'
        )

    ])
