#!/usr/bin/env python3

import math
import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw

class FakeAckermannOdom(Node):
    def __init__(self):
        super().__init__('fake_ackermann_odom')
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheelbase', 0.90)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f'Opened serial port {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.timer = self.create_timer(0.02, self.read_and_publish)

    def read_and_publish(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            if not line.startswith('FB,'):
                return

            parts = line.split(',')
            if len(parts) != 5:
                self.get_logger().warn(f'Bad line: {line}')
                return

            throttle_cmd = int(parts[1])
            steer_pwm = int(parts[2])
            v = float(parts[3])
            steer_angle = float(parts[4])

            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

            if dt <= 0.0 or dt > 0.5:
                return

            omega = 0.0
            if abs(self.wheelbase) > 1e-6:
                omega = v * math.tan(steer_angle) / self.wheelbase

            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += omega * dt

            qz, qw = yaw_to_quaternion(self.yaw)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = omega

            odom.pose.covariance = [
                0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ]

            odom.twist.covariance = [
                0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ]

            self.odom_pub.publish(odom)

            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.base_frame
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f'Read/parse error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FakeAckermannOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()