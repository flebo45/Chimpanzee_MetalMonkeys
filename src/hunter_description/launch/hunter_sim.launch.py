import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'hunter_description'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Processa il file Xacro
    # Trova il file Xacro sorgente
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    # Lo processa (risolve le variabili matematiche) e lo trasforma in puro XML (URDF)
    doc = xacro.process_file(xacro_file)
    # Salva la stringa XML risultante nella variabile robot_desc
    robot_desc = doc.toxml()

    # 1b. Trova il file della PALLINA
    ball_file = os.path.join(pkg_share, 'urdf', 'rc_ball.sdf')

    # 1c. Trova il file del mondo (Hexagon)
    world_file = os.path.join(pkg_share, 'worlds', 'hexagon_world.sdf')

    # 2. Avvia Gazebo, includendo il launch file standard di ros_gz_sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # Passa gli argomenti:
        # '-r': Run immediato (play)
        # world_file: Carica subito il mondo
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 3. Nodo che chiede a Gazebo di spawnare il robot
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'isr_bot', 
                   '-topic', 'robot_description',  # Dove leggere l'URDF (pubblicato dopo)
                   '-x', '-4.0', '-y', '0.0', '-z', '0.2'], # Posizione Start Robot, coordinate iniziali
        output='screen'
    )

    # 3b. Spawna la Pallina Rossa
    spawn_ball = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'rc_ball', 
                   '-file', ball_file, 
                   '-x', '-2.0', '-y', '0.0', '-z', '0.2'], # Posizione Goal Pallina, coordinate davanti al robot
        output='screen'
    )

    # 4. Robot State Publisher (Pubblica la struttura del robot su ROS)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}, # Passa l'XML calcolato al punto 1
        {'use_sim_time': True}
        ],
    )

    # 5. Bridge (Il ponte ROS-Gazebo)
    
    # 'ros_gz_bridge' è un nodo speciale che legge il manuale definito in bridge.yaml e agisce come un "Man-in-the-Middle", senza il quale ROS e Gazebo sono sordi l'uno all'altro perché usano protocolli di comunicazione diversi.
    # crea le "tubature" di comunicazione (come, per esempio, per /cmd_vel):
    # lato ROS crea un Subscriber (ascoltatore) per /cmd_vel.
    # lato Gazebo crea un Publisher (emittente) per /cmd_vel.
    # appena il nodo di controllo sputa fuori un comando velocità, ros_gz_bridge lo cattura al volo, lo smonta (toglie l'intestazione ROS), lo ricompone (mette l'intestazione Gazebo), e lo spedisce dentro la simulazione.
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config},
        {'use_sim_time': True}
        ],
        output='screen'
    )

    # Restituisce la lista di tutte le azioni da eseguire
    return LaunchDescription([
        gazebo,                   # 1. Parte il mondo
        robot_state_publisher,    # 2. ROS impara la forma del robot
        spawn_robot,              # 3. Appare il robot
        spawn_ball,               # 4. Appare la palla
        bridge                    # 5. Si apre la comunicazione
    ]) 





