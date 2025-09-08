import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    controllers_config = os.path.join(
        get_package_share_directory('edrom_bringup'),
        'config', 'controllers.yaml'
    )

    urdf_path = os.path.join(
        get_package_share_directory('aurea_description'), 'urdf', 'aurea_urdf_pkg.urdf' # Mude se o nome for outro
    )
    robot_description = {'robot_description': xacro.process_file(urdf_path).toxml()}

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_config],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
        remappings=[('/forward_position_controller/commands', '/goal_joint_states')]
    )

    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner
    ])