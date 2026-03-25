import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from mavros_msgs.msg import OverrideRCIn

THROTTLE_FORWARD =1800
THROTTLE_STOP =1500
STEERING_STRAIGHT =1500

class CameraMotorTest(Node):
    def __init__(self):
        super().__init__('camera_motor_test')

        self.create_subscription(
            Bool,
            '/camera_stop_signal',
            self._camera_cb,
            10
        )

        self._rc_pub = self.create_publisher(
            OverrideRCIn,
            'mavros/rc/override',
            10
        )

        self._person_detected = False

        self.create_timer(0.1, self._control_loop)

        self.get_logger().info("Camera motor test is  ready")
        self.get_logger().info("Forward")
        self.get_logger().info("stop")
        self.get_logger().info("waiting for camera")


    def _camera_cb(self, msg):
        if msg.data and not self._person_detected:
            self.get_logger().warn("Stop")
        elif not msg.data and self._person_detected:
            self.get_logger().info("Clear")
        self._person_detected = msg.data

    
    def _control_loop(self):
        rc = OverrideRCIn()
        rc.channels = [65535] * 18
        rc.channels[0] = STEERING_STRAIGHT

        if self._person_detected:
            rc.channels[2] = THROTTLE_STOP
        else:
            rc.channels[2] = THROTTLE_FORWARD
        
        self._rc_pub.publish(rc)

    
    def main(args = None):
        rclpy.init(args=args)
        node = CameraMotorTest()
        try:
            rclpy.spin(node)
        
        except KeyboardInterrupt:
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[0] = STEERING_STRAIGHT
            rc.channels[2] = THROTTLE_STOP
            node._rc_pub.publish(rc)
            node.get_logger().info("Keyboard interrupt")
    
        finally:
            node.destroy_node()
            try:
                if rclpy.ok():
                    rclpy.shutdown()

            except Exception:
                pass

if __name__ == '__main__':
    main()