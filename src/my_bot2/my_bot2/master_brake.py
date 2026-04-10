import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mavros_msgs.srv import SetMode, CommandBool


class MasterBrake(Node):
    def __init__(self):
        super().__init__('master_brake')

        # --- 1. Listen to Camera (Priority 1: Emergency Stop) ---
        self.create_subscription(Bool, '/camera_stop_signal', self.camera_cb, 10)
        self.person_detected = False

        # --- 2. Listen to Nav2 (Priority 2: Path Navigation) ---
        self.create_subscription(Twist, '/cmd_vel', self.nav2_cb, 10)
        self.nav2_cmd = Twist()

        # --- 3. Publisher to Pixhawk via MAVROS ---
        self.cmd_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # --- 4. MAVROS service clients for ARM + GUIDED mode ---
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._initialized = False

        # Wait for MAVROS services to come online, then arm
        self.create_timer(3.0, self.initialize_pixhawk)

        # --- 5. Control loop: 10 Hz (MAVROS needs continuous stream) ---
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Master Brake Node Active. Waiting for Pixhawk...")

    # ------------------------------------------------------------------ #
    #  Pixhawk initialization: set GUIDED mode then ARM                   #
    # ------------------------------------------------------------------ #
    def initialize_pixhawk(self):
        if self._initialized:
            return

        # Check services are available
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("MAVROS /set_mode not available yet, retrying...")
            return
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("MAVROS /cmd/arming not available yet, retrying...")
            return

        # Set GUIDED mode
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'GUIDED'
        future_mode = self.set_mode_client.call_async(mode_req)
        future_mode.add_done_callback(self._mode_response_cb)

    def _mode_response_cb(self, future):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info("Pixhawk: GUIDED mode SET successfully.")
                # Now ARM the vehicle
                arm_req = CommandBool.Request()
                arm_req.value = True
                future_arm = self.arming_client.call_async(arm_req)
                future_arm.add_done_callback(self._arm_response_cb)
            else:
                self.get_logger().warn("Pixhawk: GUIDED mode request FAILED. Retrying in 3s...")
        except Exception as e:
            self.get_logger().error(f"Mode service call failed: {e}")

    def _arm_response_cb(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Pixhawk: ARMED successfully. Cart is ready to move!")
                self._initialized = True
            else:
                self.get_logger().warn("Pixhawk: ARM request FAILED. Is the Pixhawk ready?")
        except Exception as e:
            self.get_logger().error(f"Arming service call failed: {e}")

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #
    def camera_cb(self, msg):
        self.person_detected = msg.data

    def nav2_cb(self, msg):
        self.nav2_cmd = msg

    # ------------------------------------------------------------------ #
    #  Main control loop                                                   #
    # ------------------------------------------------------------------ #
    def control_loop(self):
        final_cmd = Twist()

        if not self._initialized:
            # Don't send movement commands until Pixhawk is armed
            self.cmd_pub.publish(final_cmd)
            return

        if self.person_detected:
            # PRIORITY 1: EMERGENCY STOP — person/cat/dog detected by camera
            final_cmd.linear.x = 0.0
            final_cmd.angular.z = 0.0
            self.get_logger().warn(
                "EMERGENCY STOP: Person/Cat/Dog detected!", throttle_duration_sec=1.0)
        else:
            # PRIORITY 2: Follow Nav2's planned path
            final_cmd = self.nav2_cmd

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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()