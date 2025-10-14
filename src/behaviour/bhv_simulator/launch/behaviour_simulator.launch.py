import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import EnvironmentVariable
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
            EnvironmentVariable('WEBOTS_HOME', default_value=''), '/lib/controller/python310:',
             EnvironmentVariable('PYTHONPATH', default_value='')
        ]
    )

    # Inicia o executável do Webots.
    # Webots irá abrir e executar o controlador "bhv_sim" definido no .wbt
    webots_process = ExecuteProcess(
        cmd=['webots', world_file],
        output='screen'
    )

    return LaunchDescription([
        python_path,
        webots_process
    ])