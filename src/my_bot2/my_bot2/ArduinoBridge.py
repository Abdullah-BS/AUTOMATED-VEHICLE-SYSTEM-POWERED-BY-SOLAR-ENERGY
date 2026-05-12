#!/usr/bin/env python3
import math
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


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
        # Measure wheelbase center-to-center between front and rear axle
        self.WHEELBASE = 0.65

        # Real maximum front wheel steering angle in radians
        # 0.52 rad ≈ 30 degrees
        self.MAX_STEER_ANGLE_RAD = 0.52

        # --- SEMI-AGGRESSIVE STEERING TUNING ---
        # Ignore very tiny steering commands to avoid servo shaking
        self.STEER_DEADBAND_RAD = 0.03

        # Gain > 1 makes steering more decisive
        self.STEER_GAIN = 1.35

        # Exponent < 1 boosts small/medium steering commands
        self.STEER_EXPONENT = 0.85

        # Optional minimum steering effect once outside deadband
        self.MIN_EFFECTIVE_STEER_NORM = 0.18

        # If speed is too low, avoid unstable division / weird steering
        self.MIN_SPEED_FOR_STEERING = 0.05

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
    def _twist_to_steering_angle(self, v, omega):
        # Ackermann conversion:
        # delta = atan(L * omega / v)
        if abs(v) < self.MIN_SPEED_FOR_STEERING or abs(omega) < 1e-4:
            return 0.0
        return math.atan((self.WHEELBASE * omega) / abs(v))

    def _apply_semi_aggressive_response(self, steer_angle):
        if abs(steer_angle) < self.STEER_DEADBAND_RAD:
            return 0.0

        sign = 1.0 if steer_angle >= 0.0 else -1.0

        norm = abs(steer_angle) / self.MAX_STEER_ANGLE_RAD
        norm = self._clamp(norm, 0.0, 1.0)

        # Semi-aggressive shaping
        norm = self.STEER_GAIN * (norm ** self.STEER_EXPONENT)

        # Add a minimum effect after leaving deadband so it reacts clearly
        if norm > 0.0:
            norm = max(norm, self.MIN_EFFECTIVE_STEER_NORM)

        norm = self._clamp(norm, 0.0, 1.0)

        return sign * norm * self.MAX_STEER_ANGLE_RAD

    def _steering_angle_to_pwm(self, steer_angle):
        steer_norm = steer_angle / self.MAX_STEER_ANGLE_RAD
        steer_norm = self._clamp(steer_norm, -1.0, 1.0)

        # REVERSED steering mapping:
        # positive angular.z => opposite side from before
        # negative angular.z => opposite side from before
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
    # ------------------------------------------------------------------
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