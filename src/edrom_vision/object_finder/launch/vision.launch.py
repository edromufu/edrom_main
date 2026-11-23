# vision.launch.py (Versão Unificada)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    # Configuração da câmera (mesmo arquivo usado pelo camera_calibrator)
    usb_cam_params = os.path.join(
        get_package_share_directory("object_finder"), "config", "usb_cam.yaml"
    )
    with open(usb_cam_params) as f:
        usb_cam_cfg = next(iter(yaml.safe_load(f).values()))["ros__parameters"]

    return LaunchDescription([

        DeclareLaunchArgument(
            "video_device",
            default_value=usb_cam_cfg["video_device"],
            description="Dispositivo da câmera USB",
        ),  

        DeclareLaunchArgument(
            "camera_name",
            default_value=usb_cam_cfg["camera_name"],
            description="Câmera usada (verri ou milare): escolhe o arquivo de calibração",
        ),

        DeclareLaunchArgument(
            "camera_info_url",
            default_value=[
                "package://object_finder/config/",
                LaunchConfiguration("camera_name"),
                "_camera-info.yaml",
            ],
            description="URL do YAML de calibração intrínseca da câmera",
        ),

        DeclareLaunchArgument(
            "image_rotation",
            default_value="180",
            description="Rotação (graus, sentido horário: 0, 90, 180 ou 270) aplicada "
                        "à imagem antes do YOLO, para compensar a câmera montada girada",
        ),

        DeclareLaunchArgument(
            "img_output",
            default_value="True",
            description="Mostra a janela com a inferência (False no robô sem display)",
        ),

        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            namespace="camera",
            output="screen",
            # usb_cam.yaml primeiro; o dict abaixo sobrescreve com os argumentos do launch
            parameters=[usb_cam_params, {
                "use_sim_time": False,
                "camera_name": LaunchConfiguration("camera_name"),
                "video_device": LaunchConfiguration("video_device"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
            }],
        ),

        Node(
            package='object_finder',
            executable='finder', 
            name='vision',
            output='screen',
            parameters=[{
                "use_sim_time": False,
                "image_rotation": ParameterValue(
                    LaunchConfiguration("image_rotation"), value_type=int),
                "img_output": ParameterValue(
                    LaunchConfiguration("img_output"), value_type=bool),
            }],
        ),
        
        Node(
            package='soccer_ipm',
            executable='ipm',
            name='soccer_ipm_node',
            output='screen',
            parameters=[{
                "use_sim_time": False,
                "output_frame": "base_footprint",
                "balls.ball_diameter": 0.14,
                # Distorção da lente corrigida nos pontos detectados (K e D do camera_info)
                "use_distortion": True,
                "goalposts.footpoint_out_of_image_threshold": 0.8,
                "robots.footpoint_out_of_image_threshold": 0.8,
            }],
            remappings=[
                ("camera_info", "/camera/camera_info"),
            ],
        ),
    ])
