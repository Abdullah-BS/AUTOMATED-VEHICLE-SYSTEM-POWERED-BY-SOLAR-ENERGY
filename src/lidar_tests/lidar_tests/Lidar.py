import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import cv2
import numpy as np

class LidarRadar(Node):
    def __init__(self):
        super().__init__('lidar_radar')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.listener_callback, 10)
        
        # --- RADAR GUI SETTINGS ---
        self.max_dist = 5.0          # Max distance to show in meters
        self.window_size = 600       # Size of the GUI window in pixels
        self.center = self.window_size // 2
        # Calculate how many pixels represents 1 meter
        self.scale = (self.window_size / 2) / self.max_dist 

        # Create the GUI window
        cv2.namedWindow("LiDAR Radar GUI", cv2.WINDOW_NORMAL)

    def listener_callback(self, msg):
        # Create a blank black image
        frame = np.zeros((self.window_size, self.window_size, 3), dtype=np.uint8)
        
        # Draw green distance rings (1m, 2m, 3m, 4m, 5m)
        for i in range(1, int(self.max_dist) + 1):
            radius = int(i * self.scale)
            cv2.circle(frame, (self.center, self.center), radius, (0, 50, 0), 1)
        
        # Draw a red dot in the center for the LiDAR itself
        cv2.circle(frame, (self.center, self.center), 4, (0, 0, 255), -1)

        angle = msg.angle_min
        for r in msg.ranges:
            # Only plot valid points within our max distance
            if not math.isinf(r) and msg.range_min < r < self.max_dist:
                
                # Math: Polar to Cartesian (Meters)
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                
                # Math: Cartesian to Pixels on the screen
                px = int(self.center + (x * self.scale))
                py = int(self.center - (y * self.scale)) # Flipped because Y pixels go down
                
                # Draw a bright green dot for the detected object
                if 0 <= px < self.window_size and 0 <= py < self.window_size:
                    cv2.circle(frame, (px, py), 2, (0, 255, 0), -1)
                    
            angle += msg.angle_increment

        # Show the image and update the GUI (1ms pause to allow drawing)
        cv2.imshow("LiDAR Radar GUI", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarRadar()
    print("\n>>> RADAR GUI LAUNCHED! Look for the new window... <<<")
    rclpy.spin(node)
    
    # Clean up the window when you hit Ctrl+C
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
