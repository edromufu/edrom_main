import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    op3_description_pkg = get_package_share_directory('op3_description')
    xacro_file_path = os.path.join(op3_description_pkg, 'urdf', 'robotis_op3.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file_path).toxml()
    rviz_config_path = os.path.join(op3_description_pkg, 'rviz', 'op3.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )

    ik_solver_node = Node(
        package='robot_motion',
        executable='ik_solver_node',
        name='ik_solver_node', # Damos um nome para o serviço ficar mais limpo
        parameters=[{'robot_description_path': xacro_file_path}]
    )

    ik_client_node = Node(
        package='robot_motion',
        executable='ik_client_node',
        name='ik_client_node', # Damos um nome para os parâmetros ficarem fáceis de achar
        namespace='ik_tester' # Usamos o mesmo namespace do código
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        ik_solver_node,
        ik_client_node
    ])