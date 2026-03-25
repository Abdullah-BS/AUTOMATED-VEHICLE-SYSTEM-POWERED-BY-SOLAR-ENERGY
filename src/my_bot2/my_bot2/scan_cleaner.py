import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
import math

class ScanCleaner(Node):
    def __init__(self):
        super().__init__('scan_cleaner')

        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan, '/scan_raw', self.cb, lidar_qos)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.angle_min_crop = 0.0         # 0°
        self.angle_max_crop = math.pi     # 180°

        self.get_logger().info("Scan cleaner: front 0°–180° only, inf removed")

    def cb(self, msg):
        cleaned = LaserScan()
        cleaned.header          = msg.header
        cleaned.angle_min       = self.angle_min_crop
        cleaned.angle_max       = self.angle_max_crop
        cleaned.angle_increment = msg.angle_increment
        cleaned.time_increment  = msg.time_increment
        cleaned.scan_time       = msg.scan_time
        cleaned.range_min       = msg.range_min
        cleaned.range_max       = msg.range_max

        start_idx = int((self.angle_min_crop - msg.angle_min) / msg.angle_increment)
        end_idx   = int((self.angle_max_crop - msg.angle_min) / msg.angle_increment)

        start_idx = max(0, start_idx)
        end_idx   = min(len(msg.ranges) - 1, end_idx)

        cleaned.ranges = [
            r if math.isfinite(r) else 0.0
            for r in msg.ranges[start_idx:end_idx + 1]
        ]
        cleaned.intensities = list(msg.intensities[start_idx:end_idx + 1]) \
            if msg.intensities else []

        self.pub.publish(cleaned)

def main(args=None):
    rclpy.init(args=args)
    node = ScanCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()