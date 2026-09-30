#!/usr/bin/env python3

import cv2
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from sar_interfaces.msg import TargetDetection
from px4_msgs.msg import VehicleLocalPosition

# Camera intrinsics
FOCAL_LENGTH = 381.36
IMG_W = 640
IMG_H = 480
CX = IMG_W / 2
CY = IMG_H / 2

BUS_CLASS = 5
CONFIDENCE_THRESHOLD = 0.5
DETECTION_TIMEOUT = 60.0

DRONE_NAMESPACES = ['drone_0', 'drone_1', 'drone_2', 'drone_3']


class DronePerceptionNode(Node):

    def __init__(self):
        super().__init__('drone_perception_node')

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLOv8n loaded')

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Store (x, y, z, yaw) per drone
        self.positions = {ns: None for ns in DRONE_NAMESPACES}
        self.detection_published = False
        self.detection_id = 0
        self.detection_timer = None

        self.detection_pub = self.create_publisher(
            TargetDetection, '/sar/target_detection', qos_pub)

        for ns in DRONE_NAMESPACES:
            self.create_subscription(
                Image,
                f'/{ns}/camera/image_raw',
                lambda msg, n=ns: self.image_cb(msg, n),
                10
            )
            self.create_subscription(
                VehicleLocalPosition,
                f'/{ns}/fmu/out/vehicle_local_position_v1',
                lambda msg, n=ns: self.position_cb(msg, n),
                qos_sub
            )

        self.get_logger().info(
            'Drone perception node ready. Watching 4 drone cameras for bus.')

    def position_cb(self, msg, ns):
        self.positions[ns] = (msg.x, msg.y, msg.z, msg.heading)

    def image_cb(self, msg, ns):
        if self.detection_published:
            return

        pos = self.positions[ns]
        if pos is None:
            return

        drone_x, drone_y, drone_z, yaw = pos
        altitude = abs(drone_z)

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=20)

        results = self.model(frame, verbose=False)

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])

                if cls != BUS_CLASS or conf < CONFIDENCE_THRESHOLD:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                px = (x1 + x2) / 2
                py = (y1 + y2) / 2

                # Forward-facing camera ray-ground-plane intersection
                # Vertical angle below horizon (positive = looking down)
                angle_down = (py - CY) / FOCAL_LENGTH

                if angle_down <= 0.01:
                    # Target is at or above horizon, can't estimate ground pos
                    self.get_logger().warn(
                        f'[{ns}] Bus detected but above horizon — skipping')
                    continue

                # Distance along ground to target
                ground_dist = altitude / angle_down

                # Horizontal pixel offset -> lateral offset at that distance
                lateral_offset = (px - CX) / FOCAL_LENGTH * ground_dist

                # Rotate into world frame using drone yaw
                world_x = drone_x + ground_dist * math.cos(yaw) \
                          - lateral_offset * math.sin(yaw)
                world_y = drone_y + ground_dist * math.sin(yaw) \
                          + lateral_offset * math.cos(yaw)

                self.get_logger().info(
                    f'[{ns}] Bus detected! conf={conf:.2f} '
                    f'pixel=({px:.0f},{py:.0f}) '
                    f'ground_dist={ground_dist:.1f}m '
                    f'world=({world_x:.2f},{world_y:.2f}) '
                    f'drone=({drone_x:.2f},{drone_y:.2f}) '
                    f'alt={altitude:.2f}m yaw={math.degrees(yaw):.1f}deg'
                )

                self.publish_detection(world_x, world_y, conf, ns)
                self.detection_published = True

                if self.detection_timer is not None:
                    self.detection_timer.cancel()
                self.detection_timer = self.create_timer(
                    DETECTION_TIMEOUT, self.reset_detection)
                return

    def reset_detection(self):
        self.get_logger().info('Detection timeout — ready to re-detect.')
        self.detection_published = False
        if self.detection_timer is not None:
            self.detection_timer.cancel()
            self.detection_timer = None

    def publish_detection(self, x, y, confidence, ns):
        msg = TargetDetection()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = 'map'
        msg.target_x = float(x)
        msg.target_y = float(y)
        msg.target_z = 0.0
        msg.confidence = confidence
        msg.label = 'bus'
        msg.detection_id = self.detection_id
        self.detection_id += 1

        self.detection_pub.publish(msg)
        self.get_logger().info(
            f'Published entrance location from {ns}: '
            f'x={x:.2f} y={y:.2f} conf={confidence:.2f}'
        )


def main():
    rclpy.init()
    node = DronePerceptionNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
