import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot2'

    # --- 1. Paths ---
    lidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch', 'sllidar_a1_launch.py'
    )

    # Official MAVROS launch path
    mavros_launch_path = os.path.join(
        get_package_share_directory('mavros'),
        'launch', 'apm.launch'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz', 'hardware_view.rviz'
    )

    # --- 2. Included Launch Files ---
    
    # MAVROS Inclusion (The fix for the stable Pixhawk link)
    mavros_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'tgt_system': '1',
            'tgt_component': '1'
        }.items()
    )

    # Lidar Inclusion
    lidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items()
    )

    # --- 3. Custom AI & Safety Nodes ---
    my_lidar_node = Node(
        package='my_bot2',
        executable='lidar_node',
        name='lidar_processor',
        output='screen'
    )

    my_camera_node = Node(
        package='my_bot2',
        executable='camera_node',
        name='camera_processor',
        output='screen'
    )

    master_brake_node = Node(
        package='my_bot2',
        executable='master_brake',
        name='master_brake',
        output='screen'
    )

    # --- 4. Hardware Drivers ---
    # Stripped back to allow auto-negotiation of the format
    camera_driver = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{
            'video_device': '/dev/video0', 
            'image_size': [640, 480]
            # Removed the forced formatting that crashed cv_bridge
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.1', '--y', '0', '--z', '0.1', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'laser', '--child-frame-id', 'camera']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # --- 5. Return Everything ---
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:57600'),
        mavros_include,
        lidar_include,
        my_lidar_node,
        my_camera_node,
        master_brake_node,
        camera_driver,
        static_tf,
        rviz_node
    ])