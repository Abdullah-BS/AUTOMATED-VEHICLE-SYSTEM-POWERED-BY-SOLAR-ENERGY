#!/usr/bin/env python3
import math
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # --- SERIAL CONFIGURATION ---
        self.serial_port = '/dev/arduino'
        self.baud_rate = 115200

        # --- SPEED / THROTTLE CONFIG ---
        self.MAX_SPEED = 1.0
        self.MAX_THROTTLE = 254

        # --- SERVO PWM RANGE ---
        self.STEER_MIN = 1200
        self.STEER_MAX = 2200
        self.STEER_CENTER = 1700

        # --- ACKERMANN GEOMETRY ---
        self.WHEELBASE = 0.65
        self.MAX_STEER_ANGLE_RAD = 0.52

        # --- STEERING TUNING ---
        self.STEER_DEADBAND_RAD = 0.03
        self.STEER_GAIN = 1.35
        self.STEER_EXPONENT = 0.85
        self.MIN_EFFECTIVE_STEER_NORM = 0.18
        self.MIN_SPEED_FOR_STEERING = 0.05

        # --- FEEDBACK OPTIONS ---
        self.INVERT_FEEDBACK_STEERING = False
        self.USE_ARDUINO_POSE_IF_AVAILABLE = False

        # --- HUMAN DETECTION FLAG ---
        self.person_detected = False

        # --- ODOM STATE ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = time.time()

        self.odom_pub = self.create_publisher(Odometry, '/odom_arduino', 10)

        # --- CONNECT TO ARDUINO ---
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.02)
            time.sleep(2)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")

            self.arduino.reset_input_buffer()
            self._send_stop()
            time.sleep(0.3)
            self.get_logger().info(f"Initialized steering to center: {self.STEER_CENTER}")

        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            raise SystemExit

        # --- SUBSCRIBERS ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/camera_stop_signal', self.camera_stop_cb, 10)

        self.last_cmd_time = time.time()
        self.failsafe_timer = self.create_timer(0.1, self.failsafe_check)
        self.serial_timer = self.create_timer(0.02, self.read_serial_feedback)

    def camera_stop_cb(self, msg: Bool):
        if msg.data and not self.person_detected:
            self.get_logger().warn('🚨 HUMAN DETECTED — Sending stop to Arduino!')
            self._send_stop()
        elif not msg.data and self.person_detected:
            self.get_logger().info('✅ Human cleared — Resuming normal control.')

        self.person_detected = msg.data

    def _clamp(self, value, low, high):
        return max(low, min(value, high))

    def _send_stop(self):
        stop_cmd = f"0,{self.STEER_CENTER}\n"
        self.arduino.write(stop_cmd.encode('utf-8'))

    def _twist_to_steering_angle(self, v, omega):
        if abs(v) < self.MIN_SPEED_FOR_STEERING or abs(omega) < 1e-4:
            return 0.0
        return math.atan((self.WHEELBASE * omega) / abs(v))

    def _apply_semi_aggressive_response(self, steer_angle):
        if abs(steer_angle) < self.STEER_DEADBAND_RAD:
            return 0.0

        sign = 1.0 if steer_angle >= 0.0 else -1.0

        norm = abs(steer_angle) / self.MAX_STEER_ANGLE_RAD
        norm = self._clamp(norm, 0.0, 1.0)

        norm = self.STEER_GAIN * (norm ** self.STEER_EXPONENT)

        if norm > 0.0:
            norm = max(norm, self.MIN_EFFECTIVE_STEER_NORM)

        norm = self._clamp(norm, 0.0, 1.0)
        return sign * norm * self.MAX_STEER_ANGLE_RAD

    def _steering_angle_to_pwm(self, steer_angle):
        steer_norm = steer_angle / self.MAX_STEER_ANGLE_RAD
        steer_norm = self._clamp(steer_norm, -1.0, 1.0)

        if steer_norm > 0.0:
            steering_pwm = int(
                self.STEER_CENTER + steer_norm * (self.STEER_MAX - self.STEER_CENTER)
            )
        elif steer_norm < 0.0:
            steering_pwm = int(
                self.STEER_CENTER - abs(steer_norm) * (self.STEER_CENTER - self.STEER_MIN)
            )
        else:
            steering_pwm = self.STEER_CENTER

        steering_pwm = self._clamp(steering_pwm, self.STEER_MIN, self.STEER_MAX)
        return steering_pwm, steer_norm

    def _yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        if self.person_detected:
            self._send_stop()
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        v = self._clamp(linear_x, -self.MAX_SPEED, self.MAX_SPEED)
        throttle_val = int((v / self.MAX_SPEED) * self.MAX_THROTTLE)
        throttle_val = self._clamp(throttle_val, -self.MAX_THROTTLE, self.MAX_THROTTLE)

        raw_steer_angle = self._twist_to_steering_angle(v, angular_z)
        final_steer_angle = self._apply_semi_aggressive_response(raw_steer_angle)

        if v >= 0.0:
            final_steer_angle = -final_steer_angle

        steering_pwm, steer_norm = self._steering_angle_to_pwm(final_steer_angle)

        command = f"{throttle_val},{steering_pwm}\n"
        self.arduino.write(command.encode('utf-8'))

        self.get_logger().info(
            f"cmd_vel: v={linear_x:.2f}, w={angular_z:.2f}, "
            f"throttle={throttle_val}, "
            f"raw_steer={math.degrees(raw_steer_angle):.1f}deg, "
            f"final_steer={math.degrees(final_steer_angle):.1f}deg, "
            f"steer_norm={steer_norm:.2f}, steer_pwm={steering_pwm}"
        )

    def read_serial_feedback(self):
        try:
            line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            if not line.startswith('FB,'):
                return

            parts = line.split(',')

            if len(parts) not in (5, 8):
                self.get_logger().warn(f"Bad feedback line: {line}")
                return

            throttle_cmd = int(parts[1])
            steer_pwm = int(parts[2])
            v_est = float(parts[3])
            steer_angle = float(parts[4])

            if self.INVERT_FEEDBACK_STEERING:
                steer_angle = -steer_angle

            now = time.time()
            dt = now - self.last_odom_time
            self.last_odom_time = now

            if dt <= 0.0 or dt > 0.5:
                return

            omega = 0.0
            if abs(self.WHEELBASE) > 1e-6:
                omega = v_est * math.tan(steer_angle) / self.WHEELBASE

            if len(parts) == 8 and self.USE_ARDUINO_POSE_IF_AVAILABLE:
                self.x = float(parts[5])
                self.y = float(parts[6])
                self.yaw = float(parts[7])
            else:
                self.x += v_est * math.cos(self.yaw) * dt
                self.y += v_est * math.sin(self.yaw) * dt
                self.yaw += omega * dt

            stamp = self.get_clock().now().to_msg()
            qz, qw = self._yaw_to_quaternion(self.yaw)

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            odom.twist.twist.linear.x = v_est
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

        except Exception as e:
            self.get_logger().warn(f"Serial feedback error: {e}")

    def failsafe_check(self):
        if self.person_detected or (time.time() - self.last_cmd_time > 0.5):
            self._send_stop()

    def destroy_node(self):
        try:
            self._send_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send_stop()
            node.arduino.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()