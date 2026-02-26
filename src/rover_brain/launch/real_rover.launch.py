import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for the REAL Jetson Orin Nano hardware."""
    
    mavros_pkg_share = get_package_share_directory('mavros')
    mavros_launch_path = os.path.join(mavros_pkg_share, 'launch', 'apm.launch')

    # CONNECTION ARGUMENT:
    # Change '/dev/ttyACM0' to your Pixhawk's USB port. 
    # Use 57600 for Telemetry pins or 115200 for USB.
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:115200', 
        description='Serial port connection to Pixhawk'
    )

    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()
    )

    commander_node = Node(
        package='rover_brain',
        executable='commander',
        name='rover_commander',
        output='screen'
    )

    # lidar_relay_node = Node(
    #     package='rover_brain',
    #     executable='lidar_relay',
    #     name='lidar_relay',
    #     output='screen'
    # )

    return LaunchDescription([
        fcu_url_arg,
        mavros_node,
        commander_node,
        lidar_relay_node
    ])