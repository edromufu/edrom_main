from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='states_routine',
            executable='aligning_body_routine',
            name='aligning_body_routine',
            output='screen'
        ),
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