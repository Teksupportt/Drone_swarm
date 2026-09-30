#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class PerceptionTest(Node):
    def __init__(self):
        super().__init__('perception_test')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.sub = self.create_subscription(
            Image, '/drone_0/camera/image_raw', self.callback, 10)
        self.get_logger().info('Perception test node started')

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = self.model(frame, verbose=False)
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                label = self.model.names[cls]
                conf = float(box.conf[0])
                if conf > 0.3:
                    self.get_logger().info(f'Detected: {label} ({conf:.2f})')

def main():
    rclpy.init()
    node = PerceptionTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
