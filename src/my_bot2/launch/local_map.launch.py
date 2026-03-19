from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # We use your simulation file directly
    sim_config = '/home/abdullah/ros2_ws/src/my_bot2/config/nav2_params.yaml'

    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            # We use your file, but OVERRIDE the simulation settings for the real cart
            parameters=[sim_config, {
                'use_sim_time': False,
                'local_costmap.ros__parameters.robot_base_frame': 'base_link',
                'local_costmap.ros__parameters.use_sim_time': False
            }]
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_local',
                    output='screen',
                    parameters=[{'use_sim_time': False,
                                 'autostart': True,
                                 'node_names': ['local_costmap'], 
                                 'bond_timeout': 0.0}] 
                )
            ]
        )
    ])
