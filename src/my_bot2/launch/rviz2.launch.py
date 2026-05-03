import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('my_bot2')    
    rviz_config = os.path.join(
        get_package_share_directory('my_bot2'),
        'rviz',
        'nav2_custom.rviz'
    )

    rvizLaunch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen'
    )

    return LaunchDescription([
    #     # rosbridge_delayed,
    #     # web_server,
    #     lidar_node,
    #     arduino_bridge_node,
    
    # #     # mavros_node,           # ← THE FIX: was commented out before
    # #    camera_node,
    # #     tf_base_to_laser,
    # #     tf_laser_to_camera,
    #     rf2o_delayed,
        # cmd_vel_bridge_delayed,
        # ekf_delayed,
#        camera_processor_node,
        # master_brake_node,
        # nav2_delayed,
        rvizLaunch,
    ])