import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    pkg_share = get_package_share_directory(package_name)
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    return LaunchDescription([
        # 1. Start Gazebo Environment
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. Spawn the Robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_bot', '-file', urdf_file],
            output='screen'
        ),

        # 3. Start Robot State Publisher (Required for TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 4. Start YOUR Vision Node (Logic)
        Node(
            package='my_bot',
            executable='camera',
            name='camera_logic'
        ),

        # 5. Start YOUR Lidar Node (Logic)
        Node(
            package='my_bot',
            executable='lidar',
            name='lidar_logic'
        )
    ])