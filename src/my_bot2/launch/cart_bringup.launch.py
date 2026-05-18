import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('my_bot2')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params_Ackermann.yaml')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')


    # 1. Lidar
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_a1',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/lidar',   # LIDAR always
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'inverted': False,
            'max_distance': 12.0,
            'enable_angle_crop_func': True,
        }],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )
    
    ekf_delayed = TimerAction(period=6.5, actions=[ekf_node])

        
        # 3. Arduino bridge
    arduino_bridge_node = Node(
        package='my_bot2',
        executable='ArduinoBridge',
        name='arduino_bridge',
        output='screen',
        emulate_tty=True
    )
   # 4. Camera
    camera_node = Node(
        package='my_bot2',
        executable='camera_node',
        name='camera_node',
        output='screen'
    )
    # 5. Static TFs
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.30', '0.0', '0.55', '3.14159', '0', '0', 'base_link', 'laser']    
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
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
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
    # ── rosbridge — bridges ROS2 topics/actions to the website via WebSocket ──
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}],
        output='screen'
    )
    # Delayed 5s so ROS2 graph is ready before the browser connects
    rosbridge_delayed = TimerAction(period=5.0, actions=[rosbridge_node])

    # ── HTTP server — serves web_manual.html to your phone/laptop ──
    # Points to the repo root where web_manual.html lives
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', "/home/ahmed/ros2_ws"],
        output='screen'
    )
    
    
    web_goal_bridge_node = Node(
        package='my_bot2',
        executable='web_goal_bridge',
        name='web_goal_bridge',
        output='screen',
        emulate_tty=True
    )

        
    # rviz_config = os.path.join(
    #     get_package_share_directory('my_bot2'),
    #     'rviz',
    #     'nav2_custom.rviz'
    # )

    # rvizLaunch = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
    #     output='screen'
    # )
    # Keyboard teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in a new terminal window
        parameters=[{
            'speed': 0.5,
            'turn': 1.0,
        }]
    )

    return LaunchDescription([
        rosbridge_delayed,
        web_server,
        lidar_node,
        arduino_bridge_node,
        web_goal_bridge_node,

    
        # mavros_node,           # ← THE FIX: was commented out before
        # camera_node,
        tf_base_to_laser,
        # tf_laser_to_camera,
        rf2o_delayed,

        ekf_delayed,
#        camera_processor_node,
        # master_brake_node,
        nav2_delayed,
        # teleop_node,
        # rvizLaunch,

    ])