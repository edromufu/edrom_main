import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Main launch file to start the entire robot system, including other launch files and individual nodes.
    """

    # --- Declare Launch Arguments (Optional - allows passing arguments from the command line) ---
    # Example: Allow overriding the camera index or simulation mode from the top level
    camera_idx_arg = DeclareLaunchArgument(
        'camera_idx', default_value='2', description='Index of the camera device'
    )
    img_output_arg = DeclareLaunchArgument(
        'img_output', default_value='False', description='Whether vision node should display image output'
    )
    imu_connected_arg = DeclareLaunchArgument(
        "imu_connected", default_value="false", description='Is IMU connected'
    )

    # --- Get Package Paths ---
    aurea_walk_pkg = get_package_share_directory('aurea_walk')
    aurea_kick_pkg = get_package_share_directory('aurea_kick') # Added for separate kick launch
    edrom_lowlevel_pkg = get_package_share_directory('edrom_lowlevel')
    object_finder_pkg = get_package_share_directory('object_finder')
    transitions_and_states_pkg = get_package_share_directory('transitions_and_states')
    vision_controller_pkg = get_package_share_directory('vision_controller')
    # Add other packages if needed (like imu_ros_arduino, sensor_observer from behaviour.launch.py)
    imu_ros_arduino_pkg = get_package_share_directory('imu_ros_arduino')
    sensor_observer_pkg = get_package_share_directory('sensor_observer')
    # bhv_simulator_pkg = get_package_share_directory('bhv_simulator') # If simulation is used

    # --- Include Other Launch Files ---

    # 1. Walking Engine & IK (Assuming walking.launch.py ONLY starts these two now)
    #    NOTE: The kick node was removed from this include, assuming it's launched separately.
    #    If your walking.launch.py *still* launches kick_node, you might get duplicate nodes.
    walking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aurea_walk_pkg, 'launch', 'walking.launch.py')
        )
        # Add launch_arguments here if walking.launch.py accepts any
    )

    # 2. Kick Node (Assuming kick.launch.py starts only the kick node)
    kick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aurea_kick_pkg, 'launch', 'kick.launch.py') # Assumed launch file name
        )
        # Add launch_arguments here if kick.launch.py accepts any
    )

    # 3. Low-Level Control (Direct Controller & Initializer)
    direct_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(edrom_lowlevel_pkg, 'launch', 'direct_control.launch.py')
        )
    )

    # 4. Behaviour Logic (StateMachine, ROSPacker, IMU)
    #    We pass down the simulation and imu arguments.
    behaviour_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(transitions_and_states_pkg, 'launch', 'behaviour.launch.py')
        ),
        launch_arguments={
            'imu_connected': LaunchConfiguration('imu_connected'),
            # Pass other arguments like imu_port if needed
        }.items()
    )

    # --- Start Individual Nodes ---

    # 5. Vision Node (Object Finder)
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(object_finder_pkg, 'launch', 'vision.launch.py')
        ),
        # Passa os argumentos definidos no topo deste arquivo para o vision.launch.py
        launch_arguments={
            'camera_idx': LaunchConfiguration('camera_idx'),
            'img_output': LaunchConfiguration('img_output'),
            'simulation': LaunchConfiguration('simulation') # Passa o simulation também, se necessário
        }.items()
    )

    # 6. Head Controller Node (Search and Track)
    head_controller_node = Node(
        package='vision_controller',
        executable='search_and_track_node', # Make sure this matches your setup.py entry_point
        name='head_controller_node',
        output='screen'
        # Add parameters here if head_controller_node needs them
    )

    # --- Assemble Launch Description ---
    ld = LaunchDescription()

    # Add declared arguments first
    ld.add_action(camera_idx_arg)
    ld.add_action(img_output_arg)
    ld.add_action(imu_connected_arg)

    # Add included launch files
    ld.add_action(walking_launch)
    ld.add_action(kick_launch)
    ld.add_action(direct_control_launch)
    ld.add_action(behaviour_launch)
    ld.add_action(vision_launch)

    # Add individual nodes
    ld.add_action(head_controller_node)

    return ld