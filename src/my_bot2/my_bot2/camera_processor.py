import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# --- THE FIX: The correct ROS 2 Python import for Sensor QoS ---
from rclpy.qos import qos_profile_sensor_data

class CameraSafetyNode(Node):
    def __init__(self):
        super().__init__('camera_safety_node')
        
        # Using the stable PyTorch model
        self.model = YOLO('yolov8n.pt').to('cpu')        
        self.target_classes = [0, 15, 16] # Person, Cat, Dog
        self.bridge = CvBridge()
        
        self.frame_count = 0
        self.process_every_n_frames = 2 # Process every 2nd frame
        
        # Subscribe using qos_profile_sensor_data (Notice: no parentheses!)
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, qos_profile_sensor_data)
        
        # Publisher for the Master Brake (Standard QoS is fine here, it's just a boolean)
        self.stop_pub = self.create_publisher(Bool, '/camera_stop_signal', 10)
        
        # Publish the annotated image using qos_profile_sensor_data
        self.annotated_pub = self.create_publisher(Image, '/image_annotated', qos_profile_sensor_data)

    def listener_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return 

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run the fast model at 320p resolution
        results = self.model(frame, stream=True, verbose=False, conf=0.6, classes=self.target_classes, imgsz=320)
        
        found_alive = False
        annotated_frame = frame.copy()

        for r in results:
            annotated_frame = r.plot()
            if len(r.boxes) > 0:
                found_alive = True

        # Publish Stop Signal
        stop_msg = Bool()
        stop_msg.data = found_alive
        self.stop_pub.publish(stop_msg)

        # Publish Annotated Image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        annotated_msg.header = msg.header  
        self.annotated_pub.publish(annotated_msg)

        if found_alive:
            self.get_logger().warn("Camera: Alive Being Detected!", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSafetyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Silently catch any double-shutdown errors
            pass

if __name__ == '__main__':
    main()