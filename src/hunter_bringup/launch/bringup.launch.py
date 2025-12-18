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
    # Include another launch file, the simulation launch file from hunter_description like it is part of this one
    # Con questo si va nel pacchetto hunter_description, si prende il file hunter_sim.launch.py e si istruisce ROS ad eseguire tutto ciò che c'è scritto lì
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'hunter_sim.launch.py')
        )
    )

    # Con questi blocchi di codice si definiscono i nodi (programmi eseguibili ROS) che verranno lanciati
    # 2. Perception (The Eyes)
    perception_node = Node(
        package='hunter_perception', # pacchetto dove si trova il codice
        executable='vision_node', # nome dello script da lanciare definito nel setup.py del pacchetto
        name='vision_node', # passiamo al nodo il file YAML dei parametri, così vision_node saprà che deve usare il modello yolov8n.pt e la soglia 0.4
        output='screen', # stampa i print e i log direttamente in questo terminale
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

    return LaunchDescription([ # dice a ROS di avviare tutte queste quattro cose insieme
        sim_launch,
        perception_node,
        control_node,
        telemetry_node,
        battery_node
    ])
