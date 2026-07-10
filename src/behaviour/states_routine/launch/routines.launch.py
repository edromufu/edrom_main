from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
    get_package_share_directory('states_routine'),
    'config',
    'twist_mux_params.yaml'
    )
    return LaunchDescription([
        # Node(
        #     package='twist_mux',
        #     executable='twist_mux',
        #     name='twist_mux',
        #     parameters=[config_file],
        # ),
        Node(
            package='states_routine',
            executable='walking_routine',
            name='walking_routine',
            output='screen'
        ),
        Node(
            package='states_routine',
            executable='kicking_routine',
            name='kicking_routine',
            output='screen'
        ),
        Node(
            package='states_routine',
            executable='idle_march_routine',
            name='idle_march_routine',
            output='screen'
        ),
        Node(
            package='states_routine',
            executable='idle_routine',
            name='idle_routine',
            output='screen'
        ),
        Node(
            package='states_routine',
            executable='getting_up_routine',
            name='getting_up_routine',
            output='screen'
        ),
        Node(
            package='states_routine',
            executable='searching_routine',
            name='searching_routine',
            output='screen'
        )
    ])