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
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # 1b. Trova il file della PALLINA
    ball_file = os.path.join(pkg_share, 'urdf', 'red_ball.sdf')

    # 1c. Definisci il file del mondo (Hexagon)
    world_file = os.path.join(pkg_share, 'worlds', 'hexagon_world.sdf')

    # 2. Avvia Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 3. Spawna il Robot
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'isr_bot', 
                   '-topic', 'robot_description', 
                   '-x', '-4.0', '-y', '0.0', '-z', '0.2'], # Posizione Start Robot
        output='screen'
    )

    # 3b. Spawna la Pallina Rossa
    spawn_ball = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'target_ball', 
                   '-file', ball_file, 
                   '-x', '-2.0', '-y', '0.0', '-z', '0.5'], # Posizione Goal Pallina
        output='screen'
    )

    # 4. Robot State Publisher (Pubblica la struttura del robot su ROS)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}, 
        {'use_sim_time': True}
        ],
    )

    # 5. Bridge (Il ponte ROS-Gazebo)
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config},
        {'use_sim_time': True}
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_ball,
        bridge
    ])