import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Encontra o caminho para a pasta 'share' pacote
    pkg_share = get_package_share_directory('bhv_simulator')
    
    # Caminho completo para o seu arquivo de mundo
    world_file = os.path.join(pkg_share, 'worlds', 'bhv_sim_world.wbt')

    # Configura as variáveis de ambiente para usar as bibliotecas do Webots
    # instalado no sistema, garantindo a compatibilidade.
 
    python_path = SetEnvironmentVariable(
        'PYTHONPATH',
        [
            EnvironmentVariable('WEBOTS_HOME', default_value='/usr/local/webots'), '/lib/controller/python:',
             EnvironmentVariable('PYTHONPATH', default_value='')
        ]
    )

    # Inicia o executável do Webots.
    # Webots irá abrir e executar o controlador "bhv_sim" definido no .wbt
    webots_process = ExecuteProcess(
        cmd=['webots', '--stdout', '--stderr', '--mode=realtime', world_file],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('planning', default_value='true'),
        DeclareLaunchArgument('sensors', default_value='false'),
        SetEnvironmentVariable('BHV_SIM_ENABLE_SENSORS', LaunchConfiguration('sensors')),
        Node(package='bhv_simulator', executable='trajectory_planner',
             condition=IfCondition(LaunchConfiguration('planning')),
             parameters=[{'use_sim_time': True}], output='screen'),
        python_path,
        webots_process
    ])