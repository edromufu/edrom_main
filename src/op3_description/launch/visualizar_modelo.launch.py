import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Encontra o pacote de descrição que você baixou
    op3_description_pkg = get_package_share_directory('op3_description')

    # Aponta para o arquivo XACRO (com nosso patch do 'base_link')
    xacro_file_path = os.path.join(op3_description_pkg, 'urdf', 'robotis_op3.urdf.xacro')

    # Converte o XACRO para o formato URDF em texto
    robot_desc = xacro.process_file(xacro_file_path).toxml()

    # Encontra o arquivo de configuração do RViz
    rviz_config_path = os.path.join(op3_description_pkg, 'rviz', 'op3.rviz')

    # --- Nós Essenciais APENAS para Visualização ---

    # 1. Publica as transformações do robô
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Fornece a GUI com sliders para dar uma pose inicial ao robô
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # 3. Inicia o RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )

    # Note que NÃO estamos iniciando nosso 'ik_solver_node' ou 'ik_client_node'

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])