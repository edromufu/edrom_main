import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    # --- Defina os nomes do seu pacote e arquivos aqui ---
    package_name = 'aurea_webots_sim'
    world_file_name = 'aurea_test.wbt' 
    robot_proto_name = 'Aurea' 
    # ----------------------------------------------------

    # path para o pkg
    pkg_path = get_package_share_directory(package_name)
    
    # Path para a world file
    world_file_path = os.path.join(pkg_path, 'worlds', world_file_name)

    # Inicia o Webots com o mundo especificado
    # Isso abre a janela do simulador
    webots = WebotsLauncher(
        world=world_file_path,
        gui=True # Mantenha como True para ver a simulação
    )

    # Inicia o driver/ponte do webots_ros2.
    webots_robot_driver = WebotsController(
        robot_name=robot_proto_name,
        parameters=[
            # O driver ainda precisa do URDF para obter os nomes e limites corretos das juntas
            {'robot_description': os.path.join(get_package_share_directory('aurea_description'), 'urdf', 'aurea.urdf')}
        ]
    )

    return LaunchDescription([
        webots,
        webots_robot_driver,

    ])
