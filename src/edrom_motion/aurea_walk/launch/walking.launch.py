import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Imports necessários para a lógica condicional
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_dir = get_package_share_directory('aurea_walk')
    
    #... (suas linhas de config comentadas)

    # --- 1. Declara o argumento 'teleop' ---
    # O valor padrão é 'False', então ele não será iniciado a menos que você peça
    declare_teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='False',
        description='Se True, inicia a ponte de teleoperação para o chute.'
    )

    # --- 2. Nós existentes ---
    walking_engine_node = Node(
        package='aurea_walk',
        executable='walking_engine_node',
        name='walking_engine_node',
        output='screen'
    )
    
    ik_node = Node(
        package='aurea_walk',
        executable='ik_node',
        name='smart_ik_node'
    )

    kick_node = Node(
        package='aurea_kick',
        executable='kick_node',
        name='kick_node',
        output='screen'
    )

    # --- 3. Nó da Ponte de Teleop (Condicional) ---
    # Este nó só será iniciado se o argumento 'teleop' for 'True'
    teleop_bridge_node = Node(
        package='aurea_kick',
        executable='teleop_kick_bridge',
        name='teleop_kick_bridge',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'True'"])
        )
    )

    # --- 4. Retorna a lista de todos os componentes ---
    return LaunchDescription([
        declare_teleop_arg,     # Adiciona o argumento à descrição
        
        walking_engine_node,    # Nó de caminhada
        ik_node,                # Nó de IK
        kick_node,              # Nó de chute
        
        teleop_bridge_node      # Nó da ponte de teleop (só inicia se teleop:=True)
    ])