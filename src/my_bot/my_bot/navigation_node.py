#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import NavSatFix
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # 1. Publisher: Send Twist commands (Linear=Throttle, Angular=Steering)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. Subscribers: Safety
        self.create_subscription(Bool, '/safety/human_detected', self.update_human_safety, 10)
        self.create_subscription(Bool, '/safety/obstacle_detected', self.update_obstacle_safety, 10)
        
        # 3. Subscribers: Navigation (GPS Target from App + Current Pos from Pixhawk)
        self.create_subscription(NavSatFix, '/gps/target', self.set_target, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.update_current_gps, 10)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.update_heading, 10)
        
        # State Variables
        self.human_detected = False
        self.obstacle_detected = False
        self.target_received = False
        
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0  # In Radians
        self.target_lat = 0.0
        self.target_lon = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Outdoor Ackermann Navigator Started.")

    def update_human_safety(self, msg):
        self.human_detected = msg.data

    def update_obstacle_safety(self, msg):
        self.obstacle_detected = msg.data

    def update_current_gps(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def update_heading(self, msg):
        # MAVROS sends 0..360 degrees. Convert to Radians for Python math.
        self.current_heading = math.radians(msg.data)

    def set_target(self, msg):
        self.target_lat = msg.latitude
        self.target_lon = msg.longitude
        self.target_received = True
        self.get_logger().info(f"New Target: {self.target_lat}, {self.target_lon}")

    def get_bearing_to_target(self):
        # Calculate the angle (bearing) from current GPS to target GPS
        d_lon = math.radians(self.target_lon - self.current_lon)
        lat1 = math.radians(self.current_lat)
        lat2 = math.radians(self.target_lat)

        y = math.sin(d_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
        bearing = math.atan2(y, x)
        
        # Normalize to 0..2PI
        return (bearing + 2 * math.pi) % (2 * math.pi)

    def control_loop(self):
        cmd = Twist()
        
        # --- 1. SAFETY CHECKS ---
        if self.human_detected:
            self.get_logger().warn("STOP: Human Detected!", throttle_duration_sec=2.0)
            self.cmd_vel_pub.publish(cmd)
            return
            
        if self.obstacle_detected:
            self.get_logger().warn("STOP: Obstacle Ahead!", throttle_duration_sec=2.0)
            self.cmd_vel_pub.publish(cmd)
            return

        # --- 2. NAVIGATION LOGIC ---
        if self.target_received:
            # A. Calculate Heading Error (Difference between where we face and where we want to go)
            target_bearing = self.get_bearing_to_target()
            heading_error = target_bearing - self.current_heading

            # Normalize error to shortest path (-PI to +PI)
            if heading_error > math.pi:
                heading_error -= 2 * math.pi
            elif heading_error < -math.pi:
                heading_error += 2 * math.pi

            # B. Check Distance (Are we there yet?)
            # Rough approximation: 1 degree lat approx 111km. 
            # 0.00005 degrees is approx 5 meters.
            dist_sq = (self.target_lat - self.current_lat)**2 + (self.target_lon - self.current_lon)**2
            if dist_sq < (0.00005**2):
                self.get_logger().info("Target Reached!", throttle_duration_sec=2.0)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # C. DRIVE!
                cmd.linear.x = 1.0  # Throttle (m/s)
                # Proportional Controller for Steering
                # If error is large, steer hard. If error is small, steer gently.
                cmd.angular.z = 1.0 * heading_error 
                
            self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()