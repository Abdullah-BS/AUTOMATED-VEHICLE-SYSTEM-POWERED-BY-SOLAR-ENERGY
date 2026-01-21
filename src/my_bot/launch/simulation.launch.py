import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    robot_description = open(urdf_file).read()

    # Use Gazebo Classic launch from gazebo_ros (Humble + gazebo11)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={
            # You can point to a custom world here if you add worlds/*.world
            # 'world': os.path.join(pkg_share, 'worlds', 'empty.world'),
            'verbose': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawn after Gazebo is up (avoids race conditions)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_bot', '-topic', 'robot_description', '-z', '0.1'],
        output='screen',
    )

    camera_node = Node(
        package='my_bot',
        executable='camera',
        name='camera_logic',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    lidar_node = Node(
        package='my_bot',
        executable='lidar',
        name='lidar_logic',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    navigation_node = Node(
        package='my_bot',
        executable='navigator',
        name='navigation_logic',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_entity]),
        camera_node,
        lidar_node,
        navigation_node,
    ])
