import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('my_bot2')
    nav2_params = os.path.join(pkg, 'config', 'hardware_nav2_params.yaml')

    # 1. Lidar — publishes to /scan_raw
    # 1. Lidar — publishes to /scan_raw
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_a1',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'inverted': False,
            'angle_compensate': True,
            'max_distance': 8.0,    # ← cap max range, prevents inf
            'angle_min': -1.5708,   # -90°
            'angle_max':  1.5708,   # +90°
        }],
        
        remappings=[('scan', '/scan_raw')],   # ← ADD THIS LINE HERE

        output='screen'
    )
    

    # 3. MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[{
            'fcu_url': '/dev/ttyACM0:57600',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
        }],
        output='screen'
    )

    # 4. Camera
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera',
        }],
        output='screen'
    )

    # 5. Static TF
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['1.0', '0', '0.5', '0', '0', '0', 'base_link', 'laser']
    )

    tf_laser_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_camera_tf',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'laser', 'camera']
    )

    # 6. RF2O Odometry — delayed 6s
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'freq': 10.0,
        }],
        output='screen'
    )

    rf2o_delayed = TimerAction(
        period=6.0,
        actions=[rf2o_node]
    )

    # 7. Camera Safety Node
    camera_processor_node = Node(
        package='my_bot2',
        executable='camera_node',
        name='camera_safety_node',
        output='screen'
    )

    # 8. Master Brake
    master_brake_node = Node(
        package='my_bot2',
        executable='master_brake',
        name='master_brake',
        output='screen'
    )

    # 9. Nav2 Stack — delayed 8s
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[nav2_params],
        output='screen'
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav2_params],
        output='screen'
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav2_params],
        output='screen'
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav2_params],
        output='screen'
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_params],
        output='screen'
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav2_params],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
            ]
        }],
        output='screen'
    )

    nav2_delayed = TimerAction(
        period=8.0,
        actions=[
            map_server,
            amcl,
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]
    )

    return LaunchDescription([
        lidar_node,
        mavros_node,
        camera_node,
        tf_base_to_laser,
        tf_laser_to_camera,
        rf2o_delayed,
        camera_processor_node,
        master_brake_node,
        nav2_delayed,
    ])