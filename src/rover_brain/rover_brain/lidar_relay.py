#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarRelay(Node):
    def __init__(self):
        super().__init__('lidar_relay')
        
        # 1. SUBSCRIBER: Read the raw data from the physical LiDAR
        # (Assuming your LiDAR driver publishes to the standard '/scan' topic)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_cb, 
            10)
        
        # 2. PUBLISHER: Send the LiDAR data to MAVROS
        # MAVROS automatically translates this into the MAVLink OBSTACLE_DISTANCE message
        self.obstacle_pub = self.create_publisher(
            LaserScan, 
            '/mavros/obstacle/send', 
            10)
        
        self.get_logger().info("Lidar Relay Started. Feeding Pixhawk's BendyRuler...")

    def scan_cb(self, msg):
        # NOTE: If your LiDAR sees part of your car's own chassis, 
        # you would write code here to filter out those specific angles.
        
        # For now, we simply forward the entire laser scan directly to the Pixhawk
        self.obstacle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()