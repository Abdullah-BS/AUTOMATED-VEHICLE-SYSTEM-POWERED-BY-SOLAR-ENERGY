import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('my_bot2')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    world = os.path.join(pkg, 'worlds', 'city_street.world')
    urdf  = os.path.join(pkg, 'urdf', 'robot.urdf')

    rviz_config = os.path.join(pkg, 'rviz', 'slam_nav2.rviz')

    
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg, 'maps', 'my_map.yaml')  # <-- change name

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': open(urdf).read()}]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_bot2', '-topic', 'robot_description', '-z', '0.1'],
        output='screen'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True',
            'map': map_yaml,
            'params_file': params,
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo,
        TimerAction(period=2.0, actions=[rsp]),
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=12.0, actions=[nav2_bringup]),
        TimerAction(period=13.0, actions=[rviz]),
    ])