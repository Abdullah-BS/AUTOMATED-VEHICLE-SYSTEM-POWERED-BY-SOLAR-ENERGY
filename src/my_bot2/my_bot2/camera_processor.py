import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import json
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class CameraYoloNode(Node):
    def __init__(self):
        super().__init__('camera_yolo_node')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos)
        self.detection_pub = self.create_publisher(String, '/yolo/detections', 10)
        self.stop_pub = self.create_publisher(Bool, '/camera_stop_signal', 10)

        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Human detection settings
        self.confidence_threshold = 0.1   # min confidence to count as human
        self.frame_count = 0
        self.last_annotated = None
        self.human_was_detected = False

        self.timer = self.create_timer(1 / 30, self.timer_callback)
        self.get_logger().info('Camera YOLO node started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        self.frame_count += 1

        # Run YOLO every 2nd frame to save CPU
        if self.frame_count % 2 == 0:
            results = self.model(frame, verbose=False)
            annotated = results[0].plot()
            self.last_annotated = annotated

            # Build detections list + check for humans
            detections = []
            human_found = False

            for box in results[0].boxes:
                class_name = self.model.names[int(box.cls)]
                confidence = float(box.conf)
                detections.append({
                    'class': class_name,
                    'confidence': confidence,
                    'bbox': box.xyxy[0].tolist()
                })
                if class_name == 'person' and confidence >= self.confidence_threshold:
                    human_found = True

            # Publish detections JSON
            self.detection_pub.publish(String(data=json.dumps(detections)))

            # Publish stop signal
            self.stop_pub.publish(Bool(data=human_found))

            # Log only on state change (avoid spam)
            if human_found and not self.human_was_detected:
                self.get_logger().warn('🚨 HUMAN DETECTED — Stop signal sent!')
            elif not human_found and self.human_was_detected:
                self.get_logger().info('✅ Human cleared — Stop signal released.')

            self.human_was_detected = human_found

        else:
            # Use last annotated frame on skipped frames
            annotated = self.last_annotated if self.last_annotated is not None else frame

        # Always publish image every frame for smooth video
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera'
        self.image_pub.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()