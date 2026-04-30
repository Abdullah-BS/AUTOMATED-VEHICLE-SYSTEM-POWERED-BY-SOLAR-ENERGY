#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # --- 1. CONFIGURATION ---
        self.serial_port = '/dev/ttyUSB0'  
        self.baud_rate = 115200
        
        # Vehicle Physical Limits 
        self.WHEELBASE = 0.8        # Distance from front to rear wheels in meters
        self.MAX_SPEED = 2.0        # Max speed in m/s 
        self.MAX_ANGLE = 0.523      # Max physical steering angle in radians (~30 deg)
        
        # Hardware PWM Limits
        self.STEER_MIN = 1300       # Max Left
        self.STEER_MAX = 1975       # Max Right
        # Calculate the center point for failsafe stopping
        self.STEER_CENTER = int((self.STEER_MIN + self.STEER_MAX) / 2) 
        
        # --- 2. CONNECT TO ARDUINO ---
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2) 
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            raise SystemExit

        # --- 3. SUBSCRIBER ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel', 
            self.cmd_vel_callback,
            10)
            
        self.last_cmd_time = time.time()
        self.timer = self.create_timer(0.1, self.failsafe_check)

    def map_value(self, x, in_min, in_max, out_min, out_max):
        """ Maps a value from one range to another (like Arduino's map() function) """
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # --- CALCULATE THROTTLE (-255 to 255) ---
        v = max(min(linear_x, self.MAX_SPEED), -self.MAX_SPEED)
        throttle_val = int((v / self.MAX_SPEED) * 255)
        
        # --- CALCULATE STEERING (1300 to 1975) ---
        if v == 0 and angular_z != 0:
            delta = self.MAX_ANGLE if angular_z > 0 else -self.MAX_ANGLE
        elif v == 0 and angular_z == 0:
            delta = 0.0
        else:
            delta = math.atan((self.WHEELBASE * angular_z) / v)
            
        # 1. Constrain angle to physical limits
        delta = max(min(delta, self.MAX_ANGLE), -self.MAX_ANGLE)
        
        # 2. Map the angle to your specific PWM bounds
        steering_pwm = int(self.map_value(delta, -self.MAX_ANGLE, self.MAX_ANGLE, self.STEER_MAX, self.STEER_MIN))
        
        # --- SEND TO ARDUINO ---
        # This is exactly where the speed/angle turns into the Arduino text format!
        command = f"{throttle_val},{steering_pwm}\n"
        self.arduino.write(command.encode('utf-8'))

    def failsafe_check(self):
        # Stop the car and center wheels if connection to Nav2 is lost
        if time.time() - self.last_cmd_time > 0.5:
            stop_cmd = f"0,{self.STEER_CENTER}\n"
            self.arduino.write(stop_cmd.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = f"0,{node.STEER_CENTER}\n"
        node.arduino.write(stop_cmd.encode('utf-8'))
        node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()