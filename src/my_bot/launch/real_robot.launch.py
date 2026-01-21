import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- HARDWARE DRIVERS ---
        
        # 1. Camera Driver
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{'framerate': 30.0}]
        ),

        # 2. Lidar Driver
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{'serial_port': '/dev/ttyUSB0'},
                        {'frame_id': 'laser_frame'}]
        ),

        # 3. MAVROS (Pixhawk Driver)
        # Configured for /dev/ttyACM0 (Standard for Jetson <-> Pixhawk USB)
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[{
                'fcu_url': '/dev/ttyACM0:57600',
                'system_id': 1,
                'component_id': 1,
                'target_system_id': 1,
                'target_component_id': 1,
            }]
        ),

        # 4. Rosbridge (Phone App Link)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),

        # --- YOUR LOGIC NODES ---

        # 5. Vision Node (Human Safety)
        Node(
            package='my_bot',
            executable='camera',
            name='camera_logic'
        ),

        # 6. Lidar Node (Obstacle Safety)
        Node(
            package='my_bot',
            executable='lidar',
            name='lidar_logic'
        ),
        
        # 7. Navigation Node (The Captain - NEW)
        Node(
            package='my_bot',
            executable='navigator',
            name='navigation_logic'
        )
    ])