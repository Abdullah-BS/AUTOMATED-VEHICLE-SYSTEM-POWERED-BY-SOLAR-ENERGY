import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mavros_msgs.srv import CommandBool, SetMode


class MasterBrake(Node):
    def __init__(self):
        super().__init__('master_brake')

        self.person_detected = False
        self.nav2_cmd = Twist()

        # --- Subscribers ---
        self.create_subscription(Bool,  '/camera_stop_signal', self.camera_cb, 10)
        self.create_subscription(Twist, '/cmd_vel',            self.nav2_cb,   10)

        # --- Publisher ---
        # cmd_vel_to_rc subscribes to /cmd_vel_safe (not /cmd_vel directly).
        # When a living being is detected, we publish a zero Twist here,
        # so cmd_vel_to_rc sends neutral RC (1500 / 1500) to Pixhawk.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        # --- MAVROS arming ---
        self._arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode,     '/mavros/set_mode')
        self._armed = False
        # Arm after 10 s so MAVROS is fully connected before we try
        self.create_timer(10.0, self._arm_once)

        # --- Control loop at 20 Hz ---
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'Master Brake ready — gating /cmd_vel → /cmd_vel_safe'
        )

    # ------------------------------------------------------------------
    def camera_cb(self, msg: Bool):
        self.person_detected = msg.data

    def nav2_cb(self, msg: Twist):
        self.nav2_cmd = msg

    # ------------------------------------------------------------------
    def _arm_once(self):
        if self._armed:
            return

        if not self._mode_client.service_is_ready():
            self.get_logger().warn('set_mode service not ready yet — retrying...')
            return

        if not self._arm_client.service_is_ready():
            self.get_logger().warn('arming service not ready yet — retrying...')
            return

        # Set MANUAL mode first
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'MANUAL'
        self._mode_client.call_async(mode_req)

        # Then arm
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self._arm_client.call_async(arm_req)
        future.add_done_callback(self._arm_done_cb)

    def _arm_done_cb(self, future):
        try:
            if future.result().success:
                self._armed = True
                self.get_logger().info('>>> ARMED via MAVROS <<<')
            else:
                self.get_logger().warn('Arm failed — will retry next cycle.')
        except Exception as e:
            self.get_logger().error(f'Arm callback error: {e}')

    # ------------------------------------------------------------------
    def control_loop(self):
        out = Twist()   # default: all zeros = full stop

        if self.person_detected:
            # Publish zero Twist → cmd_vel_to_rc outputs neutral (1500 / 1500)
            self.get_logger().warn(
                'EMERGENCY STOP — Living being detected!',
                throttle_duration_sec=1.0
            )
            # out is already zero, no need to modify
        else:
            # Pass Nav2 command through untouched
            out = self.nav2_cmd

        self.cmd_pub.publish(out)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MasterBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish one last zero before shutting down
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
