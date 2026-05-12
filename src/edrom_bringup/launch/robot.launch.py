import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Arquivo launch principal para iniciar o sistema do robô (Modo Autônomo).
    """

    # --- Declaração de Argumentos de Lançamento ---
    # Mantemos apenas os argumentos que o modo autônomo realmente precisa.
    camera_idx_arg = DeclareLaunchArgument(
        'camera_idx', default_value='2', description='Index of the camera device'
    )
    img_output_arg = DeclareLaunchArgument(
        'img_output', default_value='False', description='Whether vision node should display image output'
    )
    imu_connected_arg = DeclareLaunchArgument(
        "imu_connected", default_value="false", description='Is IMU connected'
    )

    # --- Obter Caminhos dos Pacotes ---
    aurea_walk_pkg = get_package_share_directory('aurea_walk')
    aurea_kick_pkg = get_package_share_directory('aurea_kick') 
    edrom_lowlevel_pkg = get_package_share_directory('edrom_lowlevel')
    object_finder_pkg = get_package_share_directory('object_finder')
    transitions_and_states_pkg = get_package_share_directory('transitions_and_states')
    vision_controller_pkg = get_package_share_directory('vision_controller')
    imu_ros_arduino_pkg = get_package_share_directory('imu_ros_arduino')
    sensor_observer_pkg = get_package_share_directory('sensor_observer')

    # --- Inclusão de Outros Launch Files ---

    # 1. Walking Engine, IK & Kick
    # (Assumindo que walking.launch.py inicia todos os nós de movimento)
    walking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aurea_walk_pkg, 'launch', 'walking.launch.py')
        )
        # Nenhuma opção de 'teleop' é passada
    )

    # 2. Low-Level Control (Direct Controller & Initializer)
    direct_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(edrom_lowlevel_pkg, 'launch', 'direct_control.launch.py')
        )
    )

    # 3. Lógica de Comportamento (StateMachine, ROSPacker, IMU)
    behaviour_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(transitions_and_states_pkg, 'launch', 'behaviour.launch.py')
        ),
        launch_arguments={
            'imu_connected': LaunchConfiguration('imu_connected'),
        }.items()
    )

    # 4. Nó de Visão (Object Finder)
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(object_finder_pkg, 'launch', 'vision.launch.py')
        ),
        # Argumento 'simulation' REMOVIDO
        launch_arguments={
            'camera_idx': LaunchConfiguration('camera_idx'),
            'img_output': LaunchConfiguration('img_output'),
        }.items()
    )

    # 5. Nó de Controle da Cabeça (Search and Track)
    head_controller_node = Node(
        package='vision_controller',
        executable='search_and_track_node', 
        name='head_controller_node',
        output='screen'
    )

    # --- Montagem da Descrição do Lançamento ---
    ld = LaunchDescription()

    # Adiciona os argumentos declarados
    ld.add_action(camera_idx_arg)
    ld.add_action(img_output_arg)
    ld.add_action(imu_connected_arg)
    # Argumentos 'simulation' e 'teleop' REMOVIDOS

    # Adiciona os launch files incluídos
    ld.add_action(walking_launch)
    ld.add_action(direct_control_launch)
    ld.add_action(behaviour_launch)
    ld.add_action(vision_launch)

    # Adiciona nós individuais
    ld.add_action(head_controller_node)

    return ld