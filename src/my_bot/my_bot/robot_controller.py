#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# This class IS the robot's brain
class RobotController(Node):
    def __init__(self):
        # Give the node a name (visible in rqt_graph)
        super().__init__('robot_controller')
        
        # This prints to the terminal
        self.get_logger().info("--- ROBOT 1: SYSTEM ONLINE ---")
        self.get_logger().info("Waiting for sensors...")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    
    # Keep the node alive until Ctrl+C is pressed
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()