import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    mapper_config = os.path.join(
        get_package_share_directory('my_mapper'),
        'config', 'campus_mapper_params.yaml'
    )

    return LaunchDescription([

        # 1. Lidar ? publishes raw scan to /scan_raw
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True
            }],
            remappings=[('scan', '/scan_raw')],
            output='screen'
        ),

        # 2. Scan cleaner (from my_bot2) ? removes inf, crops to front 180°, publishes /scan
        Node(
            package='my_bot2',
            executable='scan_cleaner',
            name='scan_cleaner',
            output='screen'
        ),

        # 3. TF: base_link ? laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # 4. RF2O odometry — delayed 5s so lidar + cleaner are ready
        TimerAction(period=5.0, actions=[
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
        ]),

        # 5. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[mapper_config],
            output='screen'
        ),

        # 6. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])