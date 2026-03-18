import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/lidar_steering_cmd', 10)
        
        # Set max reaction distance to just 0.5m (half a meter)
        self.safe_distance = 0.5 
        
        # --- THE TWEAK: Add a strike counter for debounce filtering ---
        self.obstacle_strikes = 0
        self.required_strikes = 3  # Must see it 3 times in a row to react

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        num_points = len(ranges)
        
        if num_points == 0:
            return

        def get_min_dist(start_deg, end_deg):
            start_idx = int((start_deg % 360) * num_points / 360)
            end_idx = int((end_deg % 360) * num_points / 360)
            
            if start_idx < end_idx:
                slice_arr = ranges[start_idx:end_idx]
            else: 
                slice_arr = ranges[start_idx:] + ranges[:end_idx]
                
            # Allow hand-testing: Anything between 5cm and 50cm!
            valid_dists = [r for r in slice_arr if 0.05 < r <= self.safe_distance]
            
            return min(valid_dists) if valid_dists else 99.0

        # --- EXTREMELY NARROW ZONES (Just 12 Degrees Total!) ---
        # Imagine a ruler pointing straight out from the sensor
        
        # --- WIDE 90-DEGREE CONE ---
        # 315 to 345 degrees (The front-right corner)
        right_zone = get_min_dist(315, 345)  
        
        # 345 to 15 degrees (Directly in front)
        center_zone = get_min_dist(345, 15)   
        
        # 15 to 45 degrees (The front-left corner)
        left_zone = get_min_dist(15, 45)
        cmd = Twist()

        if center_zone == 99.0 and left_zone == 99.0 and right_zone == 99.0:
            # Path is clear -> Drive forward, reset strikes
            self.obstacle_strikes = 0
            cmd.linear.x = 0.5   
            cmd.angular.z = 0.0  
            self.cmd_pub.publish(cmd)
        else:
            # We saw something! Increment the strike counter.
            self.obstacle_strikes += 1
            
            # Only react if we are SURE it is there (seen 3 times)
            if self.obstacle_strikes >= self.required_strikes:
                cmd.linear.x = 0.2 
                
                if left_zone > right_zone:
                    cmd.angular.z = 0.5 # Steer Left
                    l_print = f"{left_zone:.1f}" if left_zone != 99.0 else "Clear"
                    r_print = f"{right_zone:.1f}" if right_zone != 99.0 else "Clear"
                    self.get_logger().warn(f"Avoiding Obstacle! Steering LEFT (L:{l_print} R:{r_print})")
                else:
                    cmd.angular.z = -0.5 # Steer Right
                    l_print = f"{left_zone:.1f}" if left_zone != 99.0 else "Clear"
                    r_print = f"{right_zone:.1f}" if right_zone != 99.0 else "Clear"
                    self.get_logger().warn(f"Avoiding Obstacle! Steering RIGHT (L:{l_print} R:{r_print})")
                
                self.cmd_pub.publish(cmd)
            else:
                # We saw it, but we are waiting to confirm it's not a ghost point.
                # Keep driving forward for this split-second frame.
                cmd.linear.x = 0.5
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()