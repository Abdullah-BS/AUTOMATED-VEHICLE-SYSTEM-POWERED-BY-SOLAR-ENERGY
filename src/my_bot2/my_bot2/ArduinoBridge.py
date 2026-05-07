#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import time


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # --- CONFIGURATION ---
        self.serial_port = '/dev/arduino'
        self.baud_rate = 115200

        # Speed config
        self.MAX_SPEED = 1.0
        self.MAX_THROTTLE = 254

        # Full servo PWM range
        self.STEER_MIN = 1200
        self.STEER_MAX = 2200
        self.STEER_CENTER = 1700

        # Nav2 angular velocity scaling (rad/s -> full steering)
        # Match this to your Nav2 / velocity_smoother max theta velocity
        self.MAX_NAV2_ANGULAR_Z = 2.0

        # --- HUMAN DETECTION FLAG ---
        self.person_detected = False

        # --- CONNECT TO ARDUINO ---
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")

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
    def _clamp(self, value, low, high):
        return max(low, min(value, high))

    def _send_stop(self):
        stop_cmd = f"0,{self.STEER_CENTER}\n"
        self.arduino.write(stop_cmd.encode('utf-8'))

    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        if self.person_detected:
            self._send_stop()
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # THROTTLE (-254 to 254)
        v = self._clamp(linear_x, -self.MAX_SPEED, self.MAX_SPEED)
        throttle_val = int((v / self.MAX_SPEED) * self.MAX_THROTTLE)
        throttle_val = self._clamp(throttle_val, -self.MAX_THROTTLE, self.MAX_THROTTLE)

        # NAV2 angular velocity -> normalized steering command [-1, 1]
        steer_norm = angular_z / self.MAX_NAV2_ANGULAR_Z
        steer_norm = self._clamp(steer_norm, -1.0, 1.0)

        # Map full normalized range to full servo PWM range
        if steer_norm > 0.0:
            steering_pwm = int(
                self.STEER_CENTER - steer_norm * (self.STEER_CENTER - self.STEER_MIN)
            )
        elif steer_norm < 0.0:
            steering_pwm = int(
                self.STEER_CENTER + abs(steer_norm) * (self.STEER_MAX - self.STEER_CENTER)
            )
        else:
            steering_pwm = self.STEER_CENTER

        steering_pwm = self._clamp(steering_pwm, self.STEER_MIN, self.STEER_MAX)

        command = f"{throttle_val},{steering_pwm}\n"
        self.arduino.write(command.encode('utf-8'))

        self.get_logger().info(
            f"cmd_vel: v={linear_x:.2f}, w={angular_z:.2f}, "
            f"throttle={throttle_val}, steer_norm={steer_norm:.2f}, steer_pwm={steering_pwm}"
        )

    # ------------------------------------------------------------------
    def failsafe_check(self):
        if self.person_detected or (time.time() - self.last_cmd_time > 0.5):
            self._send_stop()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._send_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_stop()
        node.arduino.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()