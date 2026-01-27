import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    my_bot_share = get_package_share_directory('my_bot')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    sim_launch = os.path.join(my_bot_share, 'launch', 'simulation_nav2.launch.py')

    localization_launch = os.path.join(
        nav2_bringup_share, 'launch', 'localization_launch.py'
    )

    map_yaml = os.path.join(my_bot_share, 'maps', 'my_map.yaml')
    params_file = os.path.join(my_bot_share, 'config', 'nav2_params.yaml')

    rviz_config = os.path.join(
        nav2_bringup_share, 'rviz', 'nav2_default_view.rviz'
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml,
            'params_file': params_file,
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch)
        ),

        TimerAction(period=3.0, actions=[localization]),
        TimerAction(period=6.0, actions=[rviz]),
    ])
