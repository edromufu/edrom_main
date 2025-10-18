import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Inicia todos os launch files e nós necessários para o sistema completo do robô,
    passando parâmetros específicos para o launch file da visão.
    """

    # --- Encontra o caminho para os pacotes ---
    aurea_walk_pkg = get_package_share_directory('aurea_walk')
    edrom_lowlevel_pkg = get_package_share_directory('edrom_lowlevel')
    object_finder_pkg = get_package_share_directory('object_finder')
    transitions_and_states_pkg = get_package_share_directory('transitions_and_states')
    vision_controller_pkg = get_package_share_directory('vision_controller')

    # --- Inclusões dos Launch Files ---

    # 1. Inclui o walking.launch.py
    walking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aurea_walk_pkg, 'launch', 'walking.launch.py')
        )
    )

    # 2. Inclui o direct_control.launch.py
    direct_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(edrom_lowlevel_pkg, 'launch', 'direct_control.launch.py')
        )
    )

    # 3. Inclui o vision.launch.py COM PARÂMETROS
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(object_finder_pkg, 'launch', 'vision.launch.py')
        ),
        # Passa os parâmetros como um dicionário
        launch_arguments={
            'camera_idx': '2',
            'img_output': 'False'
        }.items() # Usa .items() para converter o dicionário
    )

    # 4. Inclui o behaviour.launch.py
    behaviour_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(transitions_and_states_pkg, 'launch', 'behaviour.launch.py')
        )
    )

    # --- Execução do Nó Individual (ros2 run) ---

    # 5. Inicia o head_controller_node
    head_controller_node = Node(
        package='vision_controller',
        executable='head_controller_node',
        name='head_controller_node',
        output='screen'
    )

    # --- Monta a Descrição de Lançamento Final ---
    return LaunchDescription([
        walking_launch,
        direct_control_launch,
        vision_launch, # Agora inclui os parâmetros
        behaviour_launch,
        head_controller_node
    ])