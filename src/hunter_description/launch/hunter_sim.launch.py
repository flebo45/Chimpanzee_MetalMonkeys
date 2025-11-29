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

    # 2. Avvia Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Spawna il Robot
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'isr_bot', '-topic', 'robot_description', '-z', '0.1'],
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
        spawn,
        bridge
    ])