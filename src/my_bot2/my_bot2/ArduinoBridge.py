#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import math
import time


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # --- CONFIGURATION ---
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200

        self.WHEELBASE = 0.8
        self.MAX_SPEED = 2.0
        self.MAX_ANGLE = 0.523

        self.STEER_MIN = 1300
        self.STEER_MAX = 2000
        self.STEER_CENTER = int((self.STEER_MIN + self.STEER_MAX) / 2)

        # --- HUMAN DETECTION FLAG ---
        self.person_detected = False

        # --- CONNECT TO ARDUINO ---
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            raise SystemExit

        # --- SUBSCRIBERS ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/camera_stop_signal', self.camera_stop_cb, 10)

        self.last_cmd_time = time.time()
        self.timer = self.create_timer(0.1, self.failsafe_check)

    # ------------------------------------------------------------------
    def camera_stop_cb(self, msg: Bool):
        if msg.data and not self.person_detected:
            self.get_logger().warn('🚨 HUMAN DETECTED — Sending stop to Arduino!')
            self._send_stop()
        elif not msg.data and self.person_detected:
            self.get_logger().info('✅ Human cleared — Resuming normal control.')
        self.person_detected = msg.data

    # ------------------------------------------------------------------
    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def _send_stop(self):
        """Send zero throttle + centered steering to Arduino immediately."""
        stop_cmd = f"0,{self.STEER_CENTER}\n"
        self.arduino.write(stop_cmd.encode('utf-8'))

    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        # Block movement if human is detected
        if self.person_detected:
            self._send_stop()
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # THROTTLE (-255 to 255)
        v = max(min(linear_x, self.MAX_SPEED), -self.MAX_SPEED)
        throttle_val = int((v / self.MAX_SPEED) * 255)

        # STEERING (1300 to 2000)
        if v == 0 and angular_z != 0:
            delta = self.MAX_ANGLE if angular_z > 0 else -self.MAX_ANGLE
        elif v == 0 and angular_z == 0:
            delta = 0.0
        else:
            delta = math.atan((self.WHEELBASE * angular_z) / v)

        delta = max(min(delta, self.MAX_ANGLE), -self.MAX_ANGLE)
        steering_pwm = int(self.map_value(
            delta, -self.MAX_ANGLE, self.MAX_ANGLE,
            self.STEER_MAX, self.STEER_MIN
        ))

        command = f"{throttle_val},{steering_pwm}\n"
        self.arduino.write(command.encode('utf-8'))

    # ------------------------------------------------------------------
    def failsafe_check(self):
        """Stop if Nav2 goes silent OR human is detected."""
        if self.person_detected or (time.time() - self.last_cmd_time > 0.5):
            self._send_stop()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_stop()        # ← fixed: was node.ser.close() (bug in original)
        node.arduino.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()