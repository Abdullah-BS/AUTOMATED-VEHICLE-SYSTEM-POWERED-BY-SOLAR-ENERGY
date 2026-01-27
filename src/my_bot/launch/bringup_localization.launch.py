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
    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
    rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')

    map_yaml = os.path.join(my_bot_share, 'maps', 'city_street.yaml')

    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml,
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),

        # Start AMCL + map_server after sim is up
        TimerAction(period=3.0, actions=[localization]),

        # RViz
        TimerAction(period=5.0, actions=[rviz]),
    ])
