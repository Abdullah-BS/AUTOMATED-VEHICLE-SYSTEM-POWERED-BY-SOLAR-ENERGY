import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('my_bot2')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # 1. Lidar
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_a1',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'inverted': False,
            'max_distance': 8.0,
            'enable_angle_crop_func': True,
        }],
        output='screen'
    )

    # 3. MAVROS — starts immediately so heartbeat is ready before the RC bridge
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[{
            'fcu_url': '/dev/ttyACM0:57600',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'time.timesync_rate': 0.0,
            'time.system_time_rate': 1.0,
            'time.timesync_mode': 'MAVLINK',
        }],
        output='screen'
    )

    
        # 3. Arduino bridge
    arduino_bridge_node = Node(
        package='my_bot2',
        executable='ArduinoBridge',
        name='arduino_bridge',
        output='screen',
        emulate_tty=True
    )
#    # 4. Camera
#    camera_node = Node(
#        package='v4l2_camera',
#        executable='v4l2_camera_node',
#        name='v4l2_camera_node',
#        parameters=[{
#            'video_device': '/dev/video0',
#            'image_size': [640, 480],
#            'camera_frame_id': 'camera',
#        }],
#        output='screen'
#    )

    # 5. Static TFs
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.5', '0.0', '0.5', '3.14159', '0', '0', 'base_link', 'laser']
    )

    tf_laser_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_camera_tf',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'laser', 'camera']
    )

    # 6. RF2O Odometry — delayed 6s (waits for lidar scan to stabilise)
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
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
        output='screen'
    )

    rf2o_delayed = TimerAction(period=6.0, actions=[rf2o_node])

#    # 7. Camera Safety Node
#    camera_processor_node = Node(
#        package='my_bot2',
#        executable='camera_node',
#        name='camera_safety_node',
#        output='screen'
#    )

    # 8. Master Brake
    master_brake_node = Node(
        package='my_bot2',
        executable='master_brake',
        name='master_brake',
        output='screen'
    )

    # 8b. cmd_vel -> Pixhawk RC-override bridge
    #     Delayed 7s — MAVROS needs ~3-5s to establish a heartbeat with the FCU.
    #     The extra margin ensures OverrideRCIn is not dropped at startup.
    cmd_vel_bridge = Node(
        package='my_bot2',
        executable='cmd_vel_to_rc',
        name='cmd_vel_to_rc',
        remappings=[('/cmd_vel', '/cmd_vel_safe')],
        parameters=[{
            'throttle_neutral_us': 1500,
            'throttle_fwd_us': 1600,
            'throttle_back_us': 1400,
            'steer_center_us': 1500,
            'steer_right_us': 2023,
            'steer_left_us': 1300,
            'max_speed_fwd': 0.3,
            'max_speed_back': 0.3,
            'cmd_timeout_sec': 0.5,
            'publish_rate_hz': 10.0,
        }],
        output='screen'
    )

    cmd_vel_bridge_delayed = TimerAction(period=7.0, actions=[cmd_vel_bridge])

   

    # 9. Nav2 Stack — delayed 8s (after MAVROS + RF2O are settled)
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
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        output='screen'
    )

    ekf_delayed = TimerAction(period=7.0, actions=[ekf_node])



    return LaunchDescription([
        lidar_node,
        arduino_bridge_node,
        # mavros_node,           # ← THE FIX: was commented out before
#        camera_node,
        tf_base_to_laser,
        tf_laser_to_camera,
        rf2o_delayed,
        # cmd_vel_bridge_delayed,
        # ekf_delayed,
#        camera_processor_node,
        # master_brake_node,
        nav2_delayed,
    ])