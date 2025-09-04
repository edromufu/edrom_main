import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    op3_description_path = get_package_share_directory('op3_description')
    xacro_file = os.path.join(op3_description_path, 'urdf', 'robotis_op3.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])
    rviz_config_file = os.path.join(op3_description_path, 'rviz', 'op3.rviz')
    
    return LaunchDescription([
        # Nó que publica o estado de todas as juntas e abre uma GUI com sliders
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        
        # Nó que constrói o modelo 3D
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Nó para iniciar o RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        ),

        # Nosso nó de cinemática com o serviço
        Node(
            package='op3_kinematics',
            executable='ik_node',
            name='ik_service_node'
        )
    ])