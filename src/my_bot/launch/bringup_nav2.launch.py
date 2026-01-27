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
    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    # Try to use Nav2's default RViz config; if missing, launch RViz without a config
    rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'scan_queue_size': 200,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
        }]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
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

        TimerAction(period=3.0, actions=[slam_node]),
        TimerAction(period=6.0, actions=[nav2]),
        TimerAction(period=7.0, actions=[rviz]),
    ])
