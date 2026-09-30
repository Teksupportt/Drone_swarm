#!/usr/bin/env python3

import math
import traceback
import threading
import os
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

# ─────────────────────────────────────────────
# Survey configuration
# ─────────────────────────────────────────────
HOUSES = [
    (30.0,  0.0),
    (-25.0, 35.0),
    (50.0,  45.0),
]
ORBIT_RADIUS = 10.0   # meters from house center
SURVEY_ALT   = -10.0  # NED (negative = up)
CAPTURE_DIR  = os.path.expanduser('~/ros/sar_ws/training_data/images')
WP_THRESHOLD = 1.0    # meters — waypoint reached tolerance
WP_HOLD_TICKS = 20    # ticks to hold before advancing


def orbit_waypoints(house_x, house_y, alt, radius=ORBIT_RADIUS):
    """8 waypoints around a house — corners + sides for full coverage."""
    d = radius * 0.707  # diagonal offset
    return [
        (house_x - radius, house_y,          alt),  # West
        (house_x - d,      house_y + d,      alt),  # NW corner
        (house_x,          house_y + radius, alt),  # North
        (house_x + d,      house_y + d,      alt),  # NE corner
        (house_x + radius, house_y,          alt),  # East
        (house_x + d,      house_y - d,      alt),  # SE corner
        (house_x,          house_y - radius, alt),  # South
        (house_x - d,      house_y - d,      alt),  # SW corner
    ]


class SurveyNode(Node):

    def __init__(self):
        super().__init__('survey_drone0')

        os.makedirs(CAPTURE_DIR, exist_ok=True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/drone_0/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/drone_0/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/drone_0/fmu/in/vehicle_command', qos)

        # Subscribers
        self.create_subscription(VehicleStatus,
            '/drone_0/fmu/out/vehicle_status_v4', self.status_cb, qos)
        self.create_subscription(VehicleLocalPosition,
            '/drone_0/fmu/out/vehicle_local_position_v1', self.position_cb, qos)
        self.create_subscription(Image,
            '/drone_0/camera/image_raw', self.image_cb, 10)

        # State
        self.arming_state = 0
        self.current_x = self.current_y = self.current_z = None
        self.start_x = self.start_y = None
        self.counter = 0
        self.wp_hold = 0
        self.state = 'WAIT_FOR_POSITION'
        self.prev_state = None

        # Build full waypoint list: orbit all 3 houses
        self.waypoints = []
        for hx, hy in HOUSES:
            self.waypoints.extend(orbit_waypoints(hx, hy, SURVEY_ALT))
        self.wp_index = 0

        # Image capture
        self.bridge = CvBridge()
        self.frame_count = 0
        self.capture_lock = threading.Lock()
        self.latest_frame = None
        self.capturing = False

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info(
            f'Survey node ready. {len(self.waypoints)} waypoints over '
            f'{len(HOUSES)} houses. Saving images to {CAPTURE_DIR}')

    # ─────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────

    def status_cb(self, msg):
        self.arming_state = msg.arming_state

    def position_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def image_cb(self, msg):
        if not self.capturing:
            return
        with self.capture_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    # ─────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────

    def ts(self):
        return self.get_clock().now().nanoseconds // 1000

    def pub_offboard(self):
        msg = OffboardControlMode()
        msg.timestamp = self.ts()
        msg.position = True
        self.offboard_pub.publish(msg)

    def pub_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = self.ts()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = yaw
        self.setpoint_pub.publish(msg)

    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.ts()
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 2  # drone_0 sysid
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.from_external = True
        self.command_pub.publish(msg)

    def arm(self):
        self.send_cmd(400, p1=1.0, p2=21196.0)

    def set_offboard(self):
        self.send_cmd(176, p1=1.0, p2=6.0)

    def land(self):
        self.send_cmd(21)

    def dist_to(self, wp):
        if self.current_x is None:
            return float('inf')
        return math.sqrt(
            (self.current_x - wp[0])**2 +
            (self.current_y - wp[1])**2 +
            (self.current_z - wp[2])**2)

    def yaw_toward(self, tx, ty):
        """Compute yaw to face a target (NED frame)."""
        return math.atan2(ty - self.current_y, tx - self.current_x)

    def save_frame(self):
        with self.capture_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        path = os.path.join(CAPTURE_DIR, f'frame_{self.frame_count:05d}.jpg')
        cv2.imwrite(path, bgr)
        self.frame_count += 1
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Captured {self.frame_count} frames')

    def log(self, new_state):
        if self.prev_state != new_state:
            self.get_logger().info(f'STATE: {self.prev_state} --> {new_state}')
            self.prev_state = new_state

    # ─────────────────────────────────────────────
    # Main loop
    # ─────────────────────────────────────────────

    def loop(self):
        try:
            self.log(self.state)
            self.pub_offboard()
            self.counter += 1

            if self.state == 'WAIT_FOR_POSITION':
                if self.current_x is None:
                    return
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.get_logger().info(
                    f'Position acquired: ({self.start_x:.2f}, {self.start_y:.2f})')
                self.state = 'PREFLIGHT'
                return

            wp = self.waypoints[min(self.wp_index, len(self.waypoints)-1)]

            # Face the current house center while flying
            house_idx = self.wp_index // 8
            if house_idx < len(HOUSES):
                hx, hy = HOUSES[house_idx]
                yaw = self.yaw_toward(hx, hy)
            else:
                yaw = 0.0

            self.pub_setpoint(*wp, yaw=yaw)

            if self.state == 'PREFLIGHT':
                if self.counter >= 40:
                    self.set_offboard()
                    self.state = 'OFFBOARD'

            elif self.state == 'OFFBOARD':
                if self.counter >= 60:
                    self.arm()
                    self.state = 'ARMED'

            elif self.state == 'ARMED':
                if self.arming_state == 2:
                    self.get_logger().info('Armed. Climbing to survey altitude...')
                    self.capturing = True
                    self.state = 'FLYING'
                elif self.counter % 20 == 0:
                    self.arm()

            elif self.state == 'FLYING':
                self.save_frame()
                dist = self.dist_to(wp)

                if self.counter % 20 == 0:
                    self.get_logger().info(
                        f'WP {self.wp_index+1}/{len(self.waypoints)} | '
                        f'dist={dist:.2f}m | frames={self.frame_count}')

                if dist < WP_THRESHOLD:
                    self.wp_hold += 1
                else:
                    self.wp_hold = 0

                if self.wp_hold > WP_HOLD_TICKS:
                    self.wp_hold = 0
                    self.wp_index += 1
                    if self.wp_index >= len(self.waypoints):
                        self.capturing = False
                        self.get_logger().info(
                            f'Survey complete. {self.frame_count} frames saved.')
                        self.state = 'DONE'

            elif self.state == 'DONE':
                self.pub_setpoint(self.start_x, self.start_y, SURVEY_ALT)
                if self.counter % 40 == 0:
                    self.get_logger().info('Returning home to land...')
                    self.land()
                    self.state = 'LANDING'

            elif self.state == 'LANDING':
                if self.counter % 40 == 0:
                    self.get_logger().info(f'Landing... z={self.current_z:.2f}')

        except Exception as e:
            self.get_logger().error(f'EXCEPTION: {e}\n{traceback.format_exc()}')


def main():
    rclpy.init()
    node = SurveyNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
