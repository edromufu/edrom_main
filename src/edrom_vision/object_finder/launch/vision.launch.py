# vision.launch.py (Versão Unificada)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # --- Argumento de Controle Principal ---
    # Este argumento decide se estamos em modo de simulação ou real.
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='False',
        description='Defina como "True" para rodar em modo de simulação'
    )

    # --- Argumentos para Ambos os Modos ---
    output_img_arg = DeclareLaunchArgument(
        'img_output',
        default_value='True', # Mudei o padrão para True, pois é mais útil para debug
        description='Exibe a janela com a imagem processada'
    )

    # --- Argumentos Específicos do Robô Real ---
    camera_idx_arg = DeclareLaunchArgument(
        'camera_idx',
        default_value='2',
        description='Índice da webcam (ex: 0, 1, ...)'
    )
    ajuste_arg = DeclareLaunchArgument(
        'ajuste',
        default_value='False',
        description='Habilita o ajuste manual de brilho no início'
    )
    bright_arg = DeclareLaunchArgument(
        'brilho',
        default_value='4',
        description='Valor inicial de brilho da câmera'
    )
    
    # --- Definição do Nó ---
    return LaunchDescription([
        sim_mode_arg,
        output_img_arg,
        camera_idx_arg,
        ajuste_arg,
        bright_arg,

        # --- NÓ PARA O ROBÔ REAL ---
        # Este nó só será lançado se 'sim_mode' for 'False' (padrão)
        Node(
            package='object_finder',
            # namespace='EDROM', # O namespace pode ser útil, mantido
            executable='finder', # Assumindo que o executável se chama 'finder'
            name='vision',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration('sim_mode')),
            parameters=[
                {'use_simulation': False}, # Parâmetro do nosso script unificado
                {'vision.img_output': LaunchConfiguration('img_output')},
                {'vision.camera_idx': LaunchConfiguration('camera_idx')},
                {'vision.ajuste': LaunchConfiguration('ajuste')},
                {'vision.brilho': LaunchConfiguration('brilho')}
            ]
        ),

        # --- NÓ PARA A SIMULAÇÃO ---
        # Este nó só será lançado se 'sim_mode' for 'True'
        Node(
            package='object_finder',
            # namespace='EDROM',
            executable='finder',
            name='vision',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('sim_mode')),
            parameters=[
                {'use_simulation': True}, # Parâmetro do nosso script unificado
                {'vision.img_output': LaunchConfiguration('img_output')}
                # Note que os parâmetros de câmera e brilho não são necessários aqui
            ],
            remappings=[
                ('/camera/image', '/AUREA/CAM/image_color')
            ]
        )
    ])
