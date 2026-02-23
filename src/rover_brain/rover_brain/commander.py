#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Message types for GPS coordinates and Pixhawk State
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State

# Service types for changing modes and arming
from mavros_msgs.srv import CommandBool, SetMode

class RoverCommander(Node):
    def __init__(self):
        super().__init__('rover_commander')
        
        # 1. SUBSCRIBER: Listen to the Pixhawk's current state
        self.state_sub = self.create_subscription(
            State, 
            '/mavros/state', 
            self.state_cb, 
            10)
        self.current_state = State()

        # 2. PUBLISHER: Send GPS Target to the Pixhawk
        self.target_pub = self.create_publisher(
            GeoPoseStamped, 
            '/mavros/setpoint_position/global', 
            10)

        # 3. SERVICES: Clients to change mode and arm the motors
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # 4. TIMER: Run the control loop every 1 second
        self.timer = self.create_timer(1.0, self.control_loop)
        
        # Target Coordinates (Example: Jeddah, Saudi Arabia)
        self.target_lat = 21.5433
        self.target_lon = 39.1728
        self.target_alt = 0.0

        self.get_logger().info("Rover Commander Started. Waiting for Pixhawk connection...")

    def state_cb(self, msg):
        # Update our local state variable whenever the Pixhawk broadcasts its status
        self.current_state = msg

    def control_loop(self):
        # STEP A: Do nothing if the Jetson hasn't connected to the Pixhawk yet
        if not self.current_state.connected:
            return

        # STEP B: Put the rover in GUIDED mode (ArduRover's autonomous computer mode)
        if self.current_state.mode != "GUIDED":
            self.get_logger().info("Requesting GUIDED mode...")
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.set_mode_client.call_async(req)
            return

        # STEP C: Arm the motors so the physical wheels can move
        if not self.current_state.armed:
            self.get_logger().info("Arming motors...")
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
            return

        # STEP D: Once Connected, GUIDED, and Armed -> Send the GPS target constantly
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Assign the coordinates
        msg.pose.position.latitude = self.target_lat
        msg.pose.position.longitude = self.target_lon
        msg.pose.position.altitude = self.target_alt
        
        # Publish the target to MAVROS
        self.target_pub.publish(msg)
        self.get_logger().info(f"Driving to: Lat {self.target_lat}, Lon {self.target_lon}")

def main(args=None):
    rclpy.init(args=args)
    node = RoverCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow graceful shutdown using Ctrl+C
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()