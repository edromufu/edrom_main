from launch import LaunchDescription
from launch_ros.actions import Node	
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Remove as declarações de argumentos da câmera e brilho, pois não são mais relevantes
    output_img = DeclareLaunchArgument('img_output', default_value='False')
    ajuste = DeclareLaunchArgument('ajuste', default_value='False')

    return LaunchDescription([
        output_img,
        ajuste,
        
        # Visão
        Node(
            package='object_finder',
            namespace='EDROM',
            executable='finder',
            name='vision',
            output='screen',
            parameters=[
                {'vision/img_output': LaunchConfiguration('img_output')},
                {'vision/ajuste' : LaunchConfiguration('ajuste')},
            ],
            emulate_tty=True,
        )
    ])
