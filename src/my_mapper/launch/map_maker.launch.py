import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    mapper_config = os.path.join(
        get_package_share_directory('my_mapper'),
        'config', 'campus_mapper_params.yaml'
    )

    # ── Lidar
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

    # ── TF base_link → laser
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # ── RF2O odometry (delayed 5s)
    rf2o_delayed = TimerAction(period=5.0, actions=[
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0
            }],
            output='screen'
        )
    ])

    # ── SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[mapper_config],
        output='screen'
    )

    # ── RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen'
    )

    # ── Arduino Bridge (motors)
    arduino_bridge_node = Node(
        package='my_bot2',
        executable='ArduinoBridge',
        name='arduino_bridge',
        output='screen',
        emulate_tty=True
    )

    # ── rosbridge WebSocket
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
    rosbridge_delayed = TimerAction(period=5.0, actions=[rosbridge_node])

    # ── HTTP server
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', '/home/ahmed/ros2_ws'],
        output='screen'
    )

    return LaunchDescription([
        lidar_node,
        tf_node,
        rf2o_delayed,
        slam_node,
        rviz_node,
        arduino_bridge_node,
        rosbridge_delayed,
        web_server,
    ])