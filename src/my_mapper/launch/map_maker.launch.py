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

        # 1. Lidar — publishes 180° front scan directly to /scan
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
                'angle_min': -1.5708,   # -90°
                'angle_max':  1.5708,   # +90°
            }],
            output='screen'
        ),

        # 2. TF: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # 3. RF2O odometry — delayed 5s so lidar is ready
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

        # 4. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[mapper_config],
            output='screen'
        ),

        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])