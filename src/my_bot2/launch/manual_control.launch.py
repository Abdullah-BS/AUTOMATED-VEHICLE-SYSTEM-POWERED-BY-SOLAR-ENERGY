import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Use the official MAVROS APM launch file (same as hardware.launch.py)
    mavros_launch_path = os.path.join(
        get_package_share_directory('mavros'),
        'launch', 'apm.launch'
    )

    return LaunchDescription([

        # 1. MAVROS — via official APM launch (fixes the duplicate node crash)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch_path),
            launch_arguments={
                'fcu_url': '/dev/ttyACM0:57600',
                'tgt_system': '1',
                'tgt_component': '1',
            }.items()
        ),

        # 2. Master Brake — delayed 4s to let MAVROS connect first
        TimerAction(period=4.0, actions=[
            Node(
                package='my_bot2',
                executable='master_brake',
                name='master_brake',
                output='screen'
            )
        ]),

        # 3. Rosbridge — delayed 4s
        TimerAction(period=4.0, actions=[
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                parameters=[{'port': 9090}],
                output='screen'
            )
        ]),

    ])