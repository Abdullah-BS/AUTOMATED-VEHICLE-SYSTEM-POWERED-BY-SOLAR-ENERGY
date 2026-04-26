#!/usr/bin/env python3
"""
ros_to_pix_mavros.py
====================
Converts /cmd_vel (Twist) → RC PWM values → /mavros/rc/override → Pixhawk.
Also arms the vehicle via MAVROS services on startup.

Channel map (confirmed from bench test):
    CH1  (index 0) = Steering servo   | 1200=full left  | 1500=center | 1800=full right
    CH3  (index 2) = Throttle (ESC)   | 1000=backward       | 1500=neutral| 1800=full forward

ROS convention:
    linear.x  > 0  = forward
    angular.z > 0  = left (CCW)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode

NO_OVERRIDE = 65535


class RosToPix(Node):
    def __init__(self):
        super().__init__('ros_to_pix_mavros')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('throttle_neutral',   1500)
        self.declare_parameter('throttle_fwd_max',   1800)
        self.declare_parameter('throttle_rev_max',   1200)
        self.declare_parameter('steer_center',       1500)
        self.declare_parameter('steer_left_max',     1200)
        self.declare_parameter('steer_right_max',    1800)
        self.declare_parameter('max_linear_speed',   0.3)   # m/s  → full throttle
        self.declare_parameter('max_angular_speed',  1.0)   # rad/s → full steer
        self.declare_parameter('cmd_timeout_sec',    0.5)   # watchdog
        self.declare_parameter('publish_rate_hz',    10.0)

        p = lambda n: self.get_parameter(n).value
        self.th_neutral  = p('throttle_neutral')
        self.th_fwd      = p('throttle_fwd_max')
        self.th_rev      = p('throttle_rev_max')
        self.st_center   = p('steer_center')
        self.st_left     = p('steer_left_max')
        self.st_right    = p('steer_right_max')
        self.max_v       = p('max_linear_speed')
        self.max_w       = p('max_angular_speed')
        self.cmd_timeout = p('cmd_timeout_sec')

        # ── State ────────────────────────────────────────────────────────
        self._armed          = False
        self._connected      = False
        self._arm_attempts   = 0
        self._last_twist     = Twist()
        self._last_cmd_time  = self.get_clock().now()

        # ── Publisher ────────────────────────────────────────────────────
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel',       self._on_cmd_vel, 10)
        self.create_subscription(State, '/mavros/state',  self._on_state,   10)

        # ── Service clients ──────────────────────────────────────────────
        self._arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode,     '/mavros/set_mode')

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(2.0,               self._arm_loop)      # arming state machine
        self.create_timer(1.0 / p('publish_rate_hz'), self._publish_rc)  # RC stream

        self.get_logger().info('ros_to_pix_mavros started — waiting for MAVROS...')

    # ── /mavros/state ────────────────────────────────────────────────────

    def _on_state(self, msg: State):
        self._connected = msg.connected
        if msg.armed and not self._armed:
            self._armed = True
            self.get_logger().info('>>> ARMED — ready to drive <<<')

    # ── Arm state machine (runs every 2 s) ───────────────────────────────

    def _arm_loop(self):
        if self._armed:
            return

        if not self._connected:
            self.get_logger().info(
                'Waiting for MAVROS connection...',
                throttle_duration_sec=4.0
            )
            return

        # Step 1 — set MANUAL mode once
        if self._arm_attempts == 0:
            if self._mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = 'MANUAL'
                self._mode_client.call_async(req)
                self.get_logger().info('Mode → MANUAL')

        # Step 2 — request arm
        if self._arm_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self._arm_client.call_async(req)
            self._arm_attempts += 1
            self.get_logger().info(f'Arm request #{self._arm_attempts} sent...')
        else:
            self.get_logger().warn(
                'Arming service not ready yet',
                throttle_duration_sec=4.0
            )

    # ── /cmd_vel callback ─────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        self._last_twist    = msg
        self._last_cmd_time = self.get_clock().now()

    # ── RC publish (runs at 10 Hz) ────────────────────────────────────────

    def _publish_rc(self):
        if not self._armed:
            return

        # Watchdog — if cmd_vel is stale → go neutral/stop
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            steer_us    = self.st_center
            throttle_us = self.th_neutral
        else:
            steer_us, throttle_us = self._twist_to_pwm(self._last_twist)

        rc = OverrideRCIn()
        rc.channels = [NO_OVERRIDE] * 18
        rc.channels[0] = int(steer_us)       # CH1 = steering
        rc.channels[2] = int(throttle_us)    # CH3 = throttle

        self.rc_pub.publish(rc)

        self.get_logger().info(
            f'CH1(steer)={int(steer_us)}  CH3(throttle)={int(throttle_us)}',
            throttle_duration_sec=0.3
        )

    # ── Twist → PWM conversion ────────────────────────────────────────────

    def _twist_to_pwm(self, twist: Twist):
        v = max(-self.max_v, min(self.max_v, twist.linear.x))
        w = max(-self.max_w, min(self.max_w, twist.angular.z))

        # Throttle: forward = higher PWM
        if v >= 0:
            frac = v / self.max_v
            throttle = self.th_neutral + frac * (self.th_fwd - self.th_neutral)
        else:
            frac = -v / self.max_v
            throttle = self.th_neutral - frac * (self.th_neutral - self.th_rev)

        # Steering: +angular.z = LEFT = lower PWM (1200)
        #           -angular.z = RIGHT = higher PWM (1800)
        frac_w = w / self.max_w
        if frac_w >= 0:   # left
            steer = self.st_center + frac_w * (self.st_left - self.st_center)
        else:             # right
            steer = self.st_center + (-frac_w) * (self.st_right - self.st_center)

        throttle = max(1000, min(2000, throttle))
        steer    = max(1000, min(2000, steer))

        return steer, throttle


# ── Entry point ───────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RosToPix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safe stop on shutdown
        try:
            stop = OverrideRCIn()
            stop.channels = [NO_OVERRIDE] * 18
            stop.channels[0] = 1500
            stop.channels[2] = 1500
            node.rc_pub.publish(stop)
            node.get_logger().info('Shutdown: RC neutralised.')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()