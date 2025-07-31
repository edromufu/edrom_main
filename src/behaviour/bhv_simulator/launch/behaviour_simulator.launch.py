import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Procura o caminho para o pacote
    pkg_share = get_package_share_directory('bhv_simulator')
    
    # Caminho completo para o mundo
    world_file = os.path.join(pkg_share, 'worlds', 'bhv_sim_world.wbt')

    # Configura o ambiente para o Webots encontrar as bibliotecas ROS e controler
    webots_env = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        [
            EnvironmentVariable('WEBOTS_HOME'), '/lib/controller:',
            EnvironmentVariable('LD_LIBRARY_PATH')
        ]
    )
    python_path = SetEnvironmentVariable(
        'PYTHONPATH',
        [
            EnvironmentVariable('WEBOTS_HOME'), '/lib/controller/python38'
        ]
    )

    # Inicia o Webots diretamente, passando o arquivo de mundo
    webots_process = ExecuteProcess(
        cmd=['webots', world_file],
        output='screen'
    )

    return LaunchDescription([
        webots_env,
        python_path,
        webots_process
    ])