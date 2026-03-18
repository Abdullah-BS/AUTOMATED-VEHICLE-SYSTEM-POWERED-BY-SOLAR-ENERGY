import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class MasterBrake(Node):
    def __init__(self):
        super().__init__('master_brake')
        
        # --- 1. Listen to the Camera (Priority 1: Life Safety) ---
        self.create_subscription(Bool, '/camera_stop_signal', self.camera_cb, 10)
        self.person_detected = False
        
        # --- 2. Listen to the LiDAR (Priority 2 & 3: Avoidance & Navigation) ---
        self.create_subscription(Twist, '/lidar_steering_cmd', self.lidar_cb, 10)
        self.lidar_cmd = Twist() # Defaults to 0.0
        
        # --- 3. Talk to the Pixhawk (The Final Command) ---
        self.cmd_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # --- 4. The Control Loop (Runs 10 times a second) ---
        # MAVROS requires a continuous stream of commands to stay in Guided/Offboard mode
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Master Brake / Sensor Fusion Active! Protecting the Golf Cart.")

    def camera_cb(self, msg):
        # Update our camera state
        self.person_detected = msg.data

    def lidar_cb(self, msg):
        # Update our LiDAR desired steering
        self.lidar_cmd = msg

    def control_loop(self):
        final_cmd = Twist()
        
        if self.person_detected:
            # PRIORITY 1: EMERGENCY BRAKE
            # Ignore the LiDAR, slam on the brakes
            final_cmd.linear.x = 0.0
            final_cmd.angular.z = 0.0
            self.get_logger().warn("EMERGENCY BRAKE: Camera detected a person/car/dog!", throttle_duration_sec=1.0)
        else:
            # PRIORITY 2 & 3: LIDAR AVOIDANCE OR CLEAR PATH
            # Since the camera is clear, pass the LiDAR's steering command directly to the wheels
            final_cmd = self.lidar_cmd
            
        # Send the final verdict to the Pixhawk
        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MasterBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()