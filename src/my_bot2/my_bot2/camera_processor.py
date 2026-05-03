import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import json
import time
from ultralytics import YOLO


class CameraYoloNode(Node):
    def __init__(self):
        super().__init__('camera_yolo_node')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.detection_pub = self.create_publisher(String, '/yolo/detections', 10)
        self.stop_pub = self.create_publisher(Bool, '/camera_stop_signal', 10)

        # Skip 0,1 (laptop camera) — start from index 2 (USB cameras)
        self.cap = None
        for index in [0,2, 3, 4, 5, 6]:
            for codec in ['MJPG', 'H264', 'YUYV']:
                cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*codec))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                if not cap.isOpened():
                    cap.release()
                    continue
                # Warm-up: discard first frames (camera needs time to start)
                for _ in range(10):
                    cap.read()
                ret, frame = cap.read()
                if ret and frame is not None and frame.mean() > 5.0:
                    self.cap = cap
                    self.get_logger().info(f'✅ Camera opened: /dev/video{index} | codec: {codec}')
                    break
                cap.release()
            if self.cap is not None:
                break

        if self.cap is None:
            self.get_logger().error('❌ No working camera found!')
            raise SystemExit

        # Full warm-up after selection
        self.get_logger().info('Warming up camera...')
        for _ in range(20):
            self.cap.read()
        self.get_logger().info('Camera ready!')

        # Detection settings
        self.confidence_threshold = 0.5
        self.cooldown_sec = 5.0

        # State machine: 'moving' | 'stopped' | 'checking'
        self.state = 'moving'
        self.stop_time = None
        self.frame_count = 0
        self.last_annotated = None

        self.timer = self.create_timer(1 / 30, self.timer_callback)
        self.get_logger().info('Camera YOLO node started')

    # ------------------------------------------------------------------
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        self.frame_count += 1

        if self.frame_count % 2 == 0:
            results = self.model(frame, verbose=False)
            annotated = results[0].plot()
            self.last_annotated = annotated

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

            self.detection_pub.publish(String(data=json.dumps(detections)))
            self._update_state(human_found)

        else:
            annotated = self.last_annotated if self.last_annotated is not None else frame

        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera'
        self.image_pub.publish(img_msg)

    # ------------------------------------------------------------------
    def _update_state(self, human_found: bool):
        now = time.time()

        if self.state == 'moving':
            if human_found:
                self.state = 'stopped'
                self.stop_time = now
                self.stop_pub.publish(Bool(data=True))
                self.get_logger().warn('🚨 HUMAN DETECTED — Stopping robot!')
            else:
                self.stop_pub.publish(Bool(data=False))

        elif self.state == 'stopped':
            self.stop_pub.publish(Bool(data=True))
            if human_found:
                self.stop_time = now
                self.get_logger().warn(
                    '⏳ Human still present — resetting cooldown.',
                    throttle_duration_sec=2.0
                )
            else:
                elapsed = now - self.stop_time
                remaining = self.cooldown_sec - elapsed
                if elapsed >= self.cooldown_sec:
                    self.state = 'checking'
                    self.get_logger().info('🔍 Cooldown done — checking if path is clear...')
                else:
                    self.get_logger().info(
                        f'⏳ Human gone — resuming in {remaining:.1f}s',
                        throttle_duration_sec=1.0
                    )

        elif self.state == 'checking':
            if human_found:
                self.state = 'stopped'
                self.stop_time = now
                self.stop_pub.publish(Bool(data=True))
                self.get_logger().warn('🚨 Human back — stopping again!')
            else:
                self.state = 'moving'
                self.stop_pub.publish(Bool(data=False))
                self.get_logger().info('✅ Path clear — resuming movement!')

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()