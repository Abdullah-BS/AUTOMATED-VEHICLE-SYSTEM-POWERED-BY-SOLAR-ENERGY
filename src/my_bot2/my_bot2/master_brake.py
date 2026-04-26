import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode


class MasterBrake(Node):
    def __init__(self):
        super().__init__('master_brake')

        self.person_detected = False
        self.nav2_cmd = Twist()

        self.create_subscription(Bool, '/camera_stop_signal', self.camera_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.nav2_cb, 10)

        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self._arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode,     '/mavros/set_mode')
        self.create_timer(2.0, self._arm_once)   # arm 2s after startup
        self._armed = False

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Master Brake ready. Using MAVROS RC override.')

    def camera_cb(self, msg):
        self.person_detected = msg.data

    def nav2_cb(self, msg):
        self.nav2_cmd = msg

    def _arm_once(self):
    if self._armed:
        return
    # Set MANUAL mode
    mode_req = SetMode.Request()
    mode_req.custom_mode = 'MANUAL'
    self._mode_client.call_async(mode_req)
    # Arm
    arm_req = CommandBool.Request()
    arm_req.value = True
    future = self._arm_client.call_async(arm_req)
    future.add_done_callback(self._arm_done_cb)

    def _arm_done_cb(self, future):
        if future.result().success:
            self._armed = True
            self.get_logger().info('>>> ARMED via MAVROS <<<')
        else:
            self.get_logger().warn('Arm failed — retrying...')

    
    def control_loop(self):
        if not self._armed:
            return

        rc = OverrideRCIn()
        rc.channels = [65535] * 18

        if self.person_detected:
            rc.channels[0] = 1500   # CH1 steering center
            rc.channels[2] = 1500   # CH3 throttle neutral
            self.get_logger().warn(
                'EMERGENCY STOP: Person detected!',
                throttle_duration_sec=1.0
            )
        else:
            linear_x = self.nav2_cmd.linear.x
            angular_z = self.nav2_cmd.angular.z

            # CH3 throttle
            throttle = int(1500 + (linear_x * 300))
            throttle = max(1300, min(1800, throttle))

            # CH1 steering
            steering = int(1500 - (angular_z * 300))
            steering = max(1300, min(1800, steering))

            rc.channels[0] = steering
            rc.channels[2] = throttle

        self.rc_pub.publish(rc)
    

def main(args=None):
    rclpy.init(args=args)
    node = MasterBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = OverrideRCIn()
        stop.channels = [65535] * 18
        stop.channels[0] = 1500
        stop.channels[2] = 1500
        node.rc_pub.publish(stop)

        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()