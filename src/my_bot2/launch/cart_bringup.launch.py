import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('my_bot2')
    nav2_params = os.path.join(pkg, 'config', 'hardware_nav2_params.yaml')

    # ---------------------------------------------------------------
    # Launch arguments for AMCL initial pose
    #   ros2 launch my_bot2 cart_bringup.launch.py x:=3.5 y:=1.2 yaw:=1.57
    # # ---------------------------------------------------------------
    # x_arg   = DeclareLaunchArgument('x',   default_value='0.0',
    #                                 description='AMCL initial pose X in map frame [m]')
    # y_arg   = DeclareLaunchArgument('y',   default_value='0.0',
    #                                 description='AMCL initial pose Y in map frame [m]')
    # yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0',
    #                                 description='AMCL initial pose yaw in map frame [rad]')

    # x_val   = LaunchConfiguration('x')
    # y_val   = LaunchConfiguration('y')
    # yaw_val = LaunchConfiguration('yaw')

    # 1. Lidar — publishes cropped scan directly to /scan (driver was patched)
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
            'max_distance': 8.0,
        }],
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
    #    Lidar is mounted on the FRONT of the cart, 0.5 m above base_link,
    #    and rotated 180° (its 0° points toward the rear of the cart).
    #    args = [x, y, z, yaw, pitch, roll, parent, child]
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
            'init_pose_from_topic': '',
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

    # 8b. cmd_vel -> Pixhawk RC-override bridge
    #     Takes Nav2's Twist output and drives RC channels 1 (steer) / 3 (throttle).
    cmd_vel_bridge = Node(
        package='my_bot2',
        executable='cmd_vel_to_rc',
        name='cmd_vel_to_rc',
        parameters=[{
            'wheelbase_m':          0.65,
            'max_steer_rad':        0.5236,   # 30 deg
            'throttle_neutral_us':  1500,
            'throttle_fwd_us':      1800,
            'throttle_back_us':     1300,
            'steer_center_us':      1500,
            'steer_right_us':       1800,
            'steer_left_us':        1300,
            'max_speed_fwd':        1.5,
            'max_speed_back':       0.7,
            'cmd_timeout_sec':      0.5,
            'publish_rate_hz':      20.0,
        }],
        output='screen'
    )

    # Delay until MAVROS is fully up (~5 s for heartbeat).
    cmd_vel_bridge_delayed = TimerAction(
        period=7.0,
        actions=[cmd_vel_bridge]
    )

    # 9. Nav2 Stack — delayed 8s
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[nav2_params],
        output='screen'
    )

    # AMCL is fed BOTH the yaml file AND the launch-arg initial pose.
    # The dict below overrides the yaml values for initial_pose.*
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            nav2_params,
            {
                'set_initial_pose': False,
                # 'initial_pose.x':   x_val,
                # 'initial_pose.y':   y_val,
                # 'initial_pose.z':   0.0,
                # 'initial_pose.yaw': yaw_val,
            }
        ],
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
        # x_arg, y_arg, yaw_arg,
        lidar_node,
        #         mavros_node,
        camera_node,
        tf_base_to_laser,
        tf_laser_to_camera,
        rf2o_delayed,
        camera_processor_node,
        master_brake_node,
        cmd_vel_bridge_delayed,
        nav2_delayed,
    ])
    