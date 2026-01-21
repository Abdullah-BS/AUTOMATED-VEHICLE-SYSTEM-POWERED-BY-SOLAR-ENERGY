#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        # Subscribe to Lidar scan
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10)
        
        # Publish Obstacle Alert
        self.publisher = self.create_publisher(Bool, '/safety/obstacle_detected', 10)
        
        self.get_logger().info("Lidar Node Started: Monitoring obstacles...")

    def scan_callback(self, msg):
        obstacle_detected = False
        min_safe_distance = 1.0  # Meters
        
        # Check all laser points
        for distance in msg.ranges:
            # Ignore errors (0.0) and infinity
            if 0.1 < distance < min_safe_distance:
                obstacle_detected = True
                break
        
        # Publish Alert
        out_msg = Bool()
        out_msg.data = obstacle_detected
        self.publisher.publish(out_msg)
        
        if obstacle_detected:
             self.get_logger().warn(f"OBSTACLE DETECTED! < {min_safe_distance}m")

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()