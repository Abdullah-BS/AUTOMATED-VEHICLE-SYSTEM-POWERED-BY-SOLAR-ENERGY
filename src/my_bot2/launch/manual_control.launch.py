import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mavros_launch_path = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'apm.launch'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch_path),
            launch_arguments={
                'fcu_url': '/dev/pixhawk:57600',
                'tgt_system': '1',
                'tgt_component': '1',
            }.items()
        ),


        TimerAction(period=2.0, actions=[
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                parameters=[{'port': 9090}],
                output='screen'
            )
        ]),
        
        
        TimerAction(period=2.0, actions=[
            Node(
                package='my_bot2',
                executable='master_brake',
                name='master_brake',
                output='screen'
            )
        ]),
    ])