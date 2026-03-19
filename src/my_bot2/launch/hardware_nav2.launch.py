from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    params_file = '/home/abdullah/ros2_ws/src/my_bot2/config/hardware_nav2_params.yaml'

    return LaunchDescription([
        # 1. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file]
        ),

        # 2. AMCL (Localization - This listens to the Lidar and figures out where you are)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),

        # 3. Planner Server (The Global Costmap)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # 4. Controller Server (The Local Costmap & DWB Driver)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        # 5. Behavior Server (Recovery spins, backing up)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),

        # 6. BT Navigator (The Boss that receives your goals)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # 7. Lifecycle Manager (Boots the entire real-world stack)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator'],
                        'bond_timeout': 0.0
                    }]
                )
            ]
        )
    ])
