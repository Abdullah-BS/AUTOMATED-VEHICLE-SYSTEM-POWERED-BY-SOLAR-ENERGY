#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


class WebGoalBridge(Node):
    def __init__(self):
        super().__init__('web_goal_bridge')

        self.destinations = {
            'engineering': {'x': 1.50, 'y': -0.50, 'yaw': 0.0},
            'cafeteria': {'x': 3.00, 'y': 1.20, 'yaw': 1.57},
            'library': {'x': -1.00, 'y': 2.50, 'yaw': 3.14},
            'mosque': {'x': 0.00, 'y': -2.00, 'yaw': -1.57},
            'charging station': {'x': 0.00, 'y': 0.00, 'yaw': 0.0}
        }

        self.subscription = self.create_subscription(
            String,
            '/web_commands',
            self.command_callback,
            10
        )

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.active_goal_handle = None

        self.get_logger().info('Web Goal Bridge is active! Listening to /web_commands...')

    def command_callback(self, msg):
        raw_command = msg.data
        command = raw_command.strip().lower()

        self.get_logger().info(f'RAW command: {repr(raw_command)}')
        self.get_logger().info(f'Normalized command: {command}')

        if command in ['cancel', 'emergency stop']:
            self.cancel_current_goal()
            return

        if command in self.destinations:
            self.send_nav_goal(command)
        else:
            self.get_logger().warn(f'Unknown command: {repr(command)}')

    def send_nav_goal(self, target_name):
        self.cancel_current_goal()

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return

        coords = self.destinations[target_name]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(coords['x'])
        goal_msg.pose.pose.position.y = float(coords['y'])
        goal_msg.pose.pose.position.z = 0.0

        yaw = float(coords['yaw'])
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending Nav2 Goal to {target_name}: x={coords["x"]}, y={coords["y"]}, yaw={coords["yaw"]}'
        )

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal request.')
            return

        self.get_logger().info('Nav2 accepted the goal! Driving...')
        self.active_goal_handle = goal_handle

    def cancel_current_goal(self):
        if self.active_goal_handle is not None:
            self.get_logger().info('Canceling current Nav2 goal...')
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = WebGoalBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()