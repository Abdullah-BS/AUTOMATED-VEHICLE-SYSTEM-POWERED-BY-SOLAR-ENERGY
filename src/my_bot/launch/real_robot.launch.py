import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- HARDWARE DRIVERS (Zone B) ---
        
        # 1. Start the Camera Driver
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{'framerate': 30.0}]
        ),

        # 2. Start the Lidar Driver
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{'serial_port': '/dev/ttyUSB0'},
                        {'frame_id': 'laser_frame'}]
        ),

        # 3. Start MAVROS (Pixhawk Driver)
        # Note: We need a config file for this later, but this starts the node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[{'fcu_url': '/dev/ttyACM0:57600'}]
        ),

        # 4. Start Rosbridge (Flutter App Link)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),

        # --- YOUR LOGIC NODES (Zone A) ---

        # 5. Start Human Detection
        Node(
            package='my_bot',
            executable='camera',
            name='camera_logic'
        ),

        # 6. Start Obstacle Detection
        Node(
            package='my_bot',
            executable='lidar',
            name='lidar_logic'
        )
    ])