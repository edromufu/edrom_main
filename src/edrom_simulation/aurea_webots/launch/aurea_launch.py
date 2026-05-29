import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('aurea_webots')
    robot_description_path = os.path.join(package_dir, 'resource', 'aurea_urdf_pkg.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'aurea_motion.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='Aurea',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
       
    behaviour_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('transitions_and_states'),
            'launch',
            'behaviour.launch.py'
        )))

    vision_onnx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('object_finder_onnx'),
            'launch',
            'vision_onnx.launch.py'
        ))
    )


    return LaunchDescription([
        webots,
        #behaviour_launch,
        my_robot_driver,
        vision_onnx_launch,
        #head_controller_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )

 
    ])
