#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel_to_rc.py
================
Bridge node that converts Nav2's geometry_msgs/Twist on /cmd_vel into a
mavros_msgs/OverrideRCIn message sent to Pixhawk via MAVROS.

Ackermann kinematics:
    steer = atan( (w * L) / v )     [rad]   (positive = left / CCW)

RC calibration (from the user's bench test):
    Throttle (channel 3, index 2 in array):
        1500 = neutral
        1800 = full forward
        1300 = full reverse
    Steering (channel 1, index 0 in array):
        1500 = centered
        1800 = full RIGHT  (CW)
        1300 = full LEFT   (CCW)

Safety:
    * Watchdog: if no /cmd_vel message within `cmd_timeout_sec`,
      output is forced to neutral (1500, 1500).
    * Output values are clamped into the calibrated range.
    * `max_speed_fwd` / `max_speed_back` cap what v values saturate the stick.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn


NEUTRAL_US = 1500
# QGroundControl / MAVROS uses 65535 for "do NOT override this channel".
CHANNEL_NO_OVERRIDE = 65535


class CmdVelToRC(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_rc')

        # ---- Parameters (override at launch time) ------------------------
        self.declare_parameter('wheelbase_m',      0.65)
        self.declare_parameter('max_steer_rad',    math.radians(30.0))

        # Throttle calibration (microseconds)
        self.declare_parameter('throttle_neutral_us', 1500)
        self.declare_parameter('throttle_fwd_us',     1800)
        self.declare_parameter('throttle_back_us',    1300)

        # Steering calibration (microseconds)
        self.declare_parameter('steer_center_us',  1500)
        self.declare_parameter('steer_right_us',   1800)
        self.declare_parameter('steer_left_us',    1300)

        # Physical stick saturation
        self.declare_parameter('max_speed_fwd',   1.5)   # m/s -> full throttle
        self.declare_parameter('max_speed_back',  0.7)   # m/s -> full reverse

        # Watchdog
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        # Channel layout (1-based in Pixhawk, 0-based in OverrideRCIn array)
        self.declare_parameter('steer_channel_index',    0)   # RC1
        self.declare_parameter('throttle_channel_index', 2)   # RC3

        # Read everything once.
        gp = lambda n: self.get_parameter(n).value
        self.L              = gp('wheelbase_m')
        self.max_steer      = gp('max_steer_rad')
        self.throttle_mid   = gp('throttle_neutral_us')
        self.throttle_fwd   = gp('throttle_fwd_us')
        self.throttle_back  = gp('throttle_back_us')
        self.steer_mid      = gp('steer_center_us')
        self.steer_right    = gp('steer_right_us')
        self.steer_left     = gp('steer_left_us')
        self.v_max_fwd      = gp('max_speed_fwd')
        self.v_max_back     = gp('max_speed_back')
        self.timeout_sec    = gp('cmd_timeout_sec')
        self.ch_steer       = gp('steer_channel_index')
        self.ch_throttle    = gp('throttle_channel_index')

        # ---- Pub / Sub ---------------------------------------------------
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.pub = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)

        self.last_cmd_time = self.get_clock().now()
        self.last_twist    = Twist()    # initialises to zero

        # Periodic publish at fixed rate (MAVROS likes a steady stream).
        period = 1.0 / gp('publish_rate_hz')
        self.timer = self.create_timer(period, self.on_tick)

        self.get_logger().info(
            f"cmd_vel_to_rc online | L={self.L} m, "
            f"max_steer={math.degrees(self.max_steer):.0f} deg, "
            f"throttle[{self.throttle_back}..{self.throttle_mid}..{self.throttle_fwd}], "
            f"steer[L{self.steer_left} M{self.steer_mid} R{self.steer_right}]")

    # ---------------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist):
        self.last_twist    = msg
        self.last_cmd_time = self.get_clock().now()

    # ---------------------------------------------------------------------
    def on_tick(self):
        # Watchdog: if the most recent /cmd_vel is too old, go neutral.
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout_sec:
            throttle_us = self.throttle_mid
            steer_us    = self.steer_mid
        else:
            throttle_us, steer_us = self.twist_to_us(self.last_twist)

        # Build OverrideRCIn (8 channels, fill the rest with "no override").
        out = OverrideRCIn()
        out.channels = [CHANNEL_NO_OVERRIDE] * 18   # MAVROS in Humble uses 18
        out.channels[self.ch_steer]    = int(steer_us)
        out.channels[self.ch_throttle] = int(throttle_us)
        self.pub.publish(out)

    # ---------------------------------------------------------------------
    def twist_to_us(self, tw: Twist):
        """Convert a Twist into (throttle_us, steer_us)."""
        v = tw.linear.x
        w = tw.angular.z

        # --- Steering (Ackermann inverse kinematics) ----------------------
        if abs(v) < 0.05:
            # At near-zero speed steering command from angular.z is ill-defined.
            # Just center the wheels so we don't crab-walk.
            steer = 0.0
        else:
            steer = math.atan((w * self.L) / v)
        # Clamp to mechanical stop.
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # Linearly map steer angle -> microseconds.
        # Right turn (CW, negative yaw, positive in RC-right direction) -> 1800
        # Left turn  (CCW, positive yaw) -> 1300
        # ROS: +yaw = left. So +steer -> left -> steer_left_us.
        frac = steer / self.max_steer         # +1.0 = full left, -1.0 = full right
        if frac >= 0.0:
            steer_us = self.steer_mid + frac * (self.steer_left - self.steer_mid)
        else:
            steer_us = self.steer_mid + (-frac) * (self.steer_right - self.steer_mid)

        # --- Throttle (asymmetric forward/reverse) ------------------------
        if v >= 0.0:
            frac_v = min(v / self.v_max_fwd, 1.0)
            throttle_us = self.throttle_mid + frac_v * (self.throttle_fwd - self.throttle_mid)
        else:
            frac_v = min(-v / self.v_max_back, 1.0)
            throttle_us = self.throttle_mid + frac_v * (self.throttle_back - self.throttle_mid)

        # Safety clamp.
        lo_t = min(self.throttle_back, self.throttle_fwd)
        hi_t = max(self.throttle_back, self.throttle_fwd)
        throttle_us = max(lo_t, min(hi_t, throttle_us))

        lo_s = min(self.steer_left, self.steer_right)
        hi_s = max(self.steer_left, self.steer_right)
        steer_us = max(lo_s, min(hi_s, steer_us))

        return throttle_us, steer_us


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort: release the sticks on shutdown.
        msg = OverrideRCIn()
        msg.channels = [CHANNEL_NO_OVERRIDE] * 18
        node.pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
