import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    mapper_pkg = get_package_share_directory('my_mapper')

    mapper_config = os.path.join(
        mapper_pkg,
        'config',
        'campus_mapper_params.yaml'
    )

    ekf_config = os.path.join(
        mapper_pkg,
        'config',
        'ekf.yaml'
    )

    # LiDAR driver
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/lidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
        }],
        output='screen'
    )

    # Static TF: base_link -> laser
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # Arduino bridge:
    # should publish ONLY /odom_arduino as nav_msgs/Odometry
    # should NOT publish odom -> base_link TF when EKF is enabled
    arduino_bridge_node = Node(
        package='my_bot2',
        executable='ArduinoBridge',
        name='arduino_bridge',
        output='screen',
        emulate_tty=True
    )

    # EKF sensor fusion:
    # fuses /odom_arduino + /lidar_odom
    # publishes /odometry/filtered and odom -> base_link TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    # SLAM Toolbox:
    # should be configured to use:
    # odom_frame: odom
    # base_frame: base_link
    # map_frame: map
    # scan_topic: /scan
    # odom_topic: /odometry/filtered
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[mapper_config],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen'
    )

    # rosbridge WebSocket
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'send_action_goals_in_new_thread': True,
            'call_services_in_new_thread': True,
            'default_call_service_timeout': 5.0,
        }],
        output='screen'
    )

    rosbridge_delayed = TimerAction(
        period=5.0,
        actions=[rosbridge_node]
    )

    # HTTP server
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', '/home/ahmed/ros2_ws'],
        output='screen'
    )

    return LaunchDescription([
        lidar_node,
        tf_laser_node,
        arduino_bridge_node,
        ekf_node,
        slam_node,
        rviz_node,
        rosbridge_delayed,
        web_server,
    ])