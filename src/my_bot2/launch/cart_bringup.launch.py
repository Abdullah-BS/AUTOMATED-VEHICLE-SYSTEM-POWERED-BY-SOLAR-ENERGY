import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot2'
    nav2_params = os.path.join(get_package_share_directory(package_name), 'config', 'hardware_nav2_params.yaml')

    # --- 1. SENSOR DRIVERS ---
    
    # Lidar (SLLidar A1)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'
        )),
        launch_arguments={'serial_port': '/dev/ttyUSB0'}.items()
    )

    # MAVROS (Pixhawk/APM Connection)
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mavros'), 'launch', 'apm.launch'
        )),
        launch_arguments={'fcu_url': '/dev/ttyACM0:57600'}.items()
    )

    # Camera
    camera_driver = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{'video_device': '/dev/video0', 'image_size': [640, 480]}]
    )

    # --- 2. THE SKELETON (TF Tree) ---
    # This is how the Lidar and Camera are "known" to the Cart
    
    # Cart Center -> Lidar (Adjust 1.0 if Lidar is further/closer to rear axle)
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['1.0', '0', '0.5', '0', '0', '0', 'base_link', 'laser']
    )

    # Lidar -> Camera (Based on your previous code)
    tf_laser_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_camera',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'laser', 'camera']
    )

    # --- 3. ODOMETRY (RF2O) ---
    # This connects 'odom' to 'base_link' using Lidar scans
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }]
    )

    # --- 4. CUSTOM NODES (Safety & AI) ---
    lidar_processor = Node(package=package_name, executable='lidar_node', name='lidar_processor')
    camera_processor = Node(package=package_name, executable='camera_node', name='camera_processor')
    master_brake = Node(package=package_name, executable='master_brake', name='master_brake')

    # --- 5. THE NAV2 BRAIN ---
    # We include all the nodes from your successful desktop test
    map_server = Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[nav2_params])
    amcl = Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[nav2_params])
    planner = Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[nav2_params])
    controller = Node(package='nav2_controller', executable='controller_server', name='controller_server', parameters=[nav2_params])
    behaviors = Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', parameters=[nav2_params])
    bt_navigator = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', parameters=[nav2_params])

    # Lifecycle Manager to boot them all
    manager = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
                }]
            )
        ]
    )

    return LaunchDescription([
        lidar_launch,
        mavros_launch,
        camera_driver,
        tf_base_to_laser,
        tf_laser_to_camera,
        rf2o_node,
        lidar_processor,
        camera_processor,
        master_brake,
        map_server,
        amcl,
        planner,
        controller,
        behaviors,
        bt_navigator,
        manager
    ])
