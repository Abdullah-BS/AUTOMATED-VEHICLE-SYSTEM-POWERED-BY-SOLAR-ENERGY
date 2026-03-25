import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mavros_msgs.msg import OverrideRCIn


class MasterBrake(Node):
    def __init__(self):
        super().__init__('master_brake')

        self.person_detected = False
        self.nav2_cmd = Twist()

        self.create_subscription(Bool, '/camera_stop_signal', self.camera_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.nav2_cb, 10)

        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Master Brake ready. Listening to /cmd_vel...")

    def camera_cb(self, msg):
        self.person_detected = msg.data

    def nav2_cb(self, msg):
        self.nav2_cmd = msg

    def control_loop(self):
        rc = OverrideRCIn()
        rc.channels = [65535] * 18

        if self.person_detected:
            rc.channels[0] = 1500
            rc.channels[2] = 1500
            self.get_logger().warn("EMERGENCY STOP: Person detected!", throttle_duration_sec=1.0)
        else:
            linear_x  = self.nav2_cmd.linear.x
            angular_z = self.nav2_cmd.angular.z

            throttle = int(1500 - (linear_x * 300))
            steering = int(1500 - (angular_z * 300))

            rc.channels[0] = max(1000, min(2000, steering))
            rc.channels[2] = max(1000, min(2000, throttle))

        self.rc_pub.publish(rc)


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