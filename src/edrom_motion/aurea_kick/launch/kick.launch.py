import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('aurea_kick'),
        'config',
        'kick_params.yaml'
    )

    declare_teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='False',
        description='Inicia a ponte de teleoperação (True/False)'
    )

    teleop_bridge_node = Node(
        package='aurea_kick',
        executable='teleop_kick_bridge',
        name='teleop_kick_bridge',
        output='screen',
        # --- 4. Condição para iniciar ---
        # Este nó SÓ será iniciado se o argumento 'teleop' for 'True'
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'True'"])
        )
    )

    return LaunchDescription([
        Node(
            package='aurea_kick',
            executable='kick_node',
            name='kick_node',
            parameters=[config],
            output='screen'
        ),
        declare_teleop_arg,
        teleop_bridge_node
    ])