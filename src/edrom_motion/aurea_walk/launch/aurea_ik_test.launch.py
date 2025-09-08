import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_name = 'aurea_description'
    pkg_path = get_package_share_directory(pkg_name)

    urdf_file_name = 'aurea_urdf_pkg.urdf'
    urdf_file_path = os.path.join(pkg_path, 'urdf', urdf_file_name)

    robot_description_content = xacro.process_file(urdf_file_path).toxml()
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'aurea.rviz')

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        ),

        Node(
            package='aurea_walk',
            executable='ik_node',
            name='smart_ik_node'
        )
    ])