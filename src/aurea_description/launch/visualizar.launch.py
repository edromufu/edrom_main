import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Defina o nome do seu pacote
    pkg_name = 'aurea_description'
    
    # 2. Encontre o caminho para o seu pacote
    pkg_path = get_package_share_directory(pkg_name)

    # 3. Defina o nome do seu arquivo URDF principal
    urdf_file_name = 'aurea_urdf_pkg.urdf' # Mude se o nome for diferente
    urdf_file_path = os.path.join(pkg_path, 'urdf', urdf_file_name)

    # 4. Processe o arquivo URDF/Xacro e o converta para uma string XML
    robot_description_content = xacro.process_file(urdf_file_path).toxml()

    # (Opcional) Caminho para o seu arquivo de configuração do RViz
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'aurea.rviz')

    # --- Definição dos Nós ---

    # Nó que publica o estado de todas as juntas e abre a GUI com sliders
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Nó que constrói o modelo 3D a partir dos joint_states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Nó para iniciar o RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] # Descomente se você tiver um .rviz salvo
    )

    # Retorna a descrição do lançamento com todos os nós
    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])