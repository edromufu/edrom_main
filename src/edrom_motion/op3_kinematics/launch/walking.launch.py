import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('op3_kinematics')
    
    config_file = os.path.join(pkg_dir, 'config', 'walking_params.yaml')

    return LaunchDescription([
        Node(
            package='op3_kinematics',
            executable='walking_engine_node',
            name='walking_engine_node',
            parameters=[config_file],
            output='screen'
        )
    ])