#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode

NO_OVERRIDE = 65535


class CmdVelToRC(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_rc')

        self.declare_parameter('throttle_neutral_us', 1500)
        self.declare_parameter('throttle_fwd_us', 1200)
        self.declare_parameter('throttle_back_us', 1800)

        self.declare_parameter('steer_center_us', 1500)
        self.declare_parameter('steer_left_us', 1200)
        self.declare_parameter('steer_right_us', 1800)

        self.declare_parameter('max_speed_fwd', 0.3)
        self.declare_parameter('max_speed_back', 0.3)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.declare_parameter('steer_channel_index', 0)
        self.declare_parameter('throttle_channel_index', 2)

        gp = lambda n: self.get_parameter(n).value
        self.throttle_mid = gp('throttle_neutral_us')
        self.throttle_fwd = gp('throttle_fwd_us')
        self.throttle_back = gp('throttle_back_us')
        self.steer_mid = gp('steer_center_us')
        self.steer_left = gp('steer_left_us')
        self.steer_right = gp('steer_right_us')
        self.v_max_fwd = gp('max_speed_fwd')
        self.v_max_back = gp('max_speed_back')
        self.timeout_sec = gp('cmd_timeout_sec')
        self.ch_steer = gp('steer_channel_index')
        self.ch_throttle = gp('throttle_channel_index')

        self._armed = False
        self._connected = False
        self._arm_attempts = 0
        self.last_cmd_time = self.get_clock().now()
        self.last_twist = Twist()
        self._current_steer = self.steer_mid
        self._steer_step = 40

        self.sub = self.create_subscription(
            Twist, '/cmd_vel_safe', self.on_cmd_vel, 10)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.on_state, 10)

        self.pub = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.create_timer(2.0, self.arm_loop)
        self.create_timer(1.0 / gp('publish_rate_hz'), self.on_tick)

        self.get_logger().info(
            f"cmd_vel_to_rc online | throttle[F{self.throttle_fwd} N{self.throttle_mid} B{self.throttle_back}] "
            f"| steer[L{self.steer_left} C{self.steer_mid} R{self.steer_right}]"
        )

    def on_state(self, msg: State):
        self._connected = msg.connected
        if msg.armed and not self._armed:
            self._armed = True
            self.get_logger().info('Vehicle armed, RC override active.')

    def arm_loop(self):
        if self._armed:
            return

        if not self._connected:
            self.get_logger().info(
                'Waiting for MAVROS connection...',
                throttle_duration_sec=4.0
            )
            return

        if self._arm_attempts == 0 and self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'MANUAL'
            self.mode_client.call_async(req)
            self.get_logger().info('Requested MANUAL mode.')

        if self.arm_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)
            self._arm_attempts += 1
            self.get_logger().info(
                f'Arm request #{self._arm_attempts} sent...',
                throttle_duration_sec=2.0
            )

    def on_cmd_vel(self, msg: Twist):
        self.last_twist = msg
        self.last_cmd_time = self.get_clock().now()

    def on_tick(self):
        if not self._armed:
            return

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout_sec:
            steer_us = self._current_steer
            throttle_us = self.throttle_mid
        else:
            steer_us, throttle_us = self.twist_to_us(self.last_twist)

        out = OverrideRCIn()
        out.channels = [NO_OVERRIDE] * 18
        out.channels[self.ch_steer] = int(steer_us)
        out.channels[self.ch_throttle] = int(throttle_us)
        self.pub.publish(out)

    def twist_to_us(self, tw: Twist):
        v = max(-self.v_max_back, min(self.v_max_fwd, tw.linear.x))
        w = tw.angular.z

        if v >= 0.0:
            frac_v = 0.0 if self.v_max_fwd <= 0.0 else min(v / self.v_max_fwd, 1.0)
            throttle_us = self.throttle_mid + frac_v * (self.throttle_fwd - self.throttle_mid)
        else:
            frac_v = 0.0 if self.v_max_back <= 0.0 else min(-v / self.v_max_back, 1.0)
            throttle_us = self.throttle_mid + frac_v * (self.throttle_back - self.throttle_mid)

        if w > 0.2:
            self._current_steer -= self._steer_step
        elif w < -0.2:
            self._current_steer += self._steer_step

        lo_s = min(self.steer_left, self.steer_right)
        hi_s = max(self.steer_left, self.steer_right)
        self._current_steer = max(lo_s, min(hi_s, self._current_steer))

        lo_t = min(self.throttle_fwd, self.throttle_back)
        hi_t = max(self.throttle_fwd, self.throttle_back)
        throttle_us = max(lo_t, min(hi_t, throttle_us))

        return int(self._current_steer), int(throttle_us)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            msg = OverrideRCIn()
            msg.channels = [NO_OVERRIDE] * 18
            msg.channels[0] = 1500
            msg.channels[2] = 1500
            node.pub.publish(msg)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()