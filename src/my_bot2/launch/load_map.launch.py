from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    map_file = '/home/abdullah/ros2_ws/src/my_mapper/maps/my_room.yaml'
    costmap_file = '/home/abdullah/ros2_ws/src/my_bot2/config/costmap.yaml'

    return LaunchDescription([
        # 1. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': False}]
        ),
        
        # 2. Static Transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),

        # 3. The Costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            output='screen',
            parameters=[costmap_file]
        ),

        # 4. Lifecycle Manager 
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[{'use_sim_time': False,
                                 'autostart': True,
                                 'node_names': ['map_server', '/costmap/costmap'],
                                 'bond_timeout': 0.0}] 
                )
            ]
        ),

        # 5. RViz
        Node(package='rviz2', executable='rviz2', name='rviz2')
    ])
