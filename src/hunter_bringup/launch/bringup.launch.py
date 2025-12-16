import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    bringup_pkg = get_package_share_directory('hunter_bringup')
    description_pkg = get_package_share_directory('hunter_description')
    
    # Config
    params_file = os.path.join(bringup_pkg, 'config', 'params.yaml')

    # 1. Simulation (The Stage)
    # Include the simulation launch file from hunter_description
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'hunter_sim.launch.py')
        )
    )

    # 2. Perception (The Eyes)
    perception_node = Node(
        package='hunter_perception',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[params_file]
    )

    # 3. Control (The Brain)
    control_node = Node(
        package='hunter_control',
        executable='control_node',
        name='behavior_tree_node',
        output='screen'
    )

    # 4. Telemetry (The Radio)
    telemetry_node = Node(
        package='hunter_telemetry',
        executable='telemetry_node',
        name='telemetry_node',
        output='screen'
    )

    #5. Battery Simulation (The Power Source)
    battery_node = Node(
        package='hunter_control',
        executable='battery_node',
        name='battery_node',
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        perception_node,
        control_node,
        telemetry_node,
        battery_node
    ])
