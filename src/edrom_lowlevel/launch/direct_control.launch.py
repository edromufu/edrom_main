import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Caminho para o seu arquivo de configuração dos motores
    config = os.path.join(
        get_package_share_directory('edrom_lowlevel'),
        'config',
        'motors_direct.yaml'
    )

    # Definição do nó do controlador
    controller_node = Node(
        package='edrom_lowlevel',
        executable='direct_controller', # Nome do executável do CMakeLists.txt
        name='direct_controller',       # Nome do nó no grafo ROS 2
        parameters=[config],
        output='screen'
    )

    return LaunchDescription([
        controller_node
    ])