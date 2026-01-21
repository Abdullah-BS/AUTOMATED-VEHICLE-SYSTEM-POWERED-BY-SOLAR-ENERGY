#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Subscribe to Camera
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.process_image, 
            10)
        
        # Publish Safety Alert
        self.publisher = self.create_publisher(Bool, '/safety/human_detected', 10)
        
        self.bridge = CvBridge()
        
        # Setup Human Detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        self.get_logger().info("Camera Node Started: Scanning for humans...")

    def process_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.resize(frame, (640, 480))
            
            # Detect Humans
            boxes, weights = self.hog.detectMultiScale(frame, winStride=(8,8))
            
            human_detected = False
            if len(boxes) > 0:
                human_detected = True
                # Draw boxes
                for (x, y, w, h) in boxes:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            
            # Publish Alert
            msg = Bool()
            msg.data = human_detected
            self.publisher.publish(msg)

            if human_detected:
                self.get_logger().warn("HUMAN DETECTED!")

            cv2.imshow("Robot Vision", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()