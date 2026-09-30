#!/usr/bin/env python3

import threading
import traceback
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

HOVER_ALT  = -10.0
MOVE_SPEED =  3.0
YAW_RATE   =  0.8
VERT_SPEED =  1.5


class ManualSurveyNode(Node):

    def __init__(self):
        super().__init__('manual_survey')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/drone_0/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/drone_0/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/drone_0/fmu/in/vehicle_command', qos)

        self.create_subscription(VehicleStatus,
            '/drone_0/fmu/out/vehicle_status_v4', self.status_cb, qos)
        self.create_subscription(VehicleLocalPosition,
            '/drone_0/fmu/out/vehicle_local_position_v1', self.position_cb, qos)

        self.arming_state = 0
        self.current_x = self.current_y = self.current_z = None
        self.current_yaw = 0.0
        self.hold_x = self.hold_y = self.hold_z = None
        self.hold_yaw = 0.0
        self.counter = 0
        self.state = 'WAIT_FOR_POSITION'
        self.prev_state = None

        self.command = 'h'
        self.command_lock = threading.Lock()
        self.running = True

        self.timer = self.create_timer(0.05, self.loop)

        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Manual survey ready.')
        self.get_logger().info(
            'w=forward  s=backward  a=yaw left  d=yaw right  '
            'r=up  f=down  h=hover  q=land')

    def input_loop(self):
        while self.running:
            try:
                cmd = input('> ').strip().lower()
                if cmd in ('w', 'a', 's', 'd', 'h', 'q', 'r', 'f'):
                    with self.command_lock:
                        self.command = cmd
                    print(f'Command: {cmd}')
                else:
                    print('Use w/a/s/d/r/f/h/q')
            except EOFError:
                break

    def get_command(self):
        with self.command_lock:
            return self.command

    def status_cb(self, msg):
        self.arming_state = msg.arming_state

    def position_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    def ts(self):
        return self.get_clock().now().nanoseconds // 1000

    def pub_offboard(self, use_velocity=False):
        msg = OffboardControlMode()
        msg.timestamp = self.ts()
        msg.position = not use_velocity
        msg.velocity = use_velocity
        self.offboard_pub.publish(msg)

    def pub_position(self, x, y, z, yaw=None):
        msg = TrajectorySetpoint()
        msg.timestamp = self.ts()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan')] * 3
        msg.yaw = float(yaw) if yaw is not None else float('nan')
        self.setpoint_pub.publish(msg)

    def pub_velocity(self, vx, vy, vz, yaw_rate=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = self.ts()
        msg.position = [float('nan')] * 3
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = float('nan')
        msg.yawspeed = float(yaw_rate)
        self.setpoint_pub.publish(msg)

    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.ts()
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 2
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.from_external = True
        self.command_pub.publish(msg)

    def arm(self):          self.send_cmd(400, p1=1.0, p2=21196.0)
    def set_offboard(self): self.send_cmd(176, p1=1.0, p2=6.0)
    def land(self):         self.send_cmd(21)

    def log(self, new_state):
        if self.prev_state != new_state:
            self.get_logger().info(f'STATE: {self.prev_state} --> {new_state}')
            self.prev_state = new_state

    def update_hold(self):
        if self.current_x is not None:
            self.hold_x = self.current_x
            self.hold_y = self.current_y
            self.hold_z = self.current_z
            self.hold_yaw = self.current_yaw

    def loop(self):
        try:
            self.log(self.state)
            self.counter += 1
            cmd = self.get_command()

            if self.state == 'WAIT_FOR_POSITION':
                self.pub_offboard()
                if self.current_x is None:
                    return
                self.state = 'PREFLIGHT'

            elif self.state == 'PREFLIGHT':
                self.pub_offboard()
                self.pub_position(self.current_x, self.current_y, HOVER_ALT)
                if self.counter >= 40:
                    self.set_offboard()
                    self.state = 'OFFBOARD'

            elif self.state == 'OFFBOARD':
                self.pub_offboard()
                self.pub_position(self.current_x, self.current_y, HOVER_ALT)
                if self.counter >= 60:
                    self.arm()
                    self.state = 'ARMED'

            elif self.state == 'ARMED':
                self.pub_offboard()
                self.pub_position(self.current_x, self.current_y, HOVER_ALT)
                if self.arming_state == 2:
                    self.get_logger().info('Armed. Climbing...')
                    self.hold_x = self.current_x
                    self.hold_y = self.current_y
                    self.hold_z = HOVER_ALT
                    self.hold_yaw = self.current_yaw
                    self.state = 'HOVERING'
                elif self.counter % 20 == 0:
                    self.arm()

            elif self.state == 'HOVERING':
                if cmd == 'q':
                    self.state = 'LANDING'
                    return

                if cmd in ('w', 's'):
                    sign = 1.0 if cmd == 'w' else -1.0
                    vx = sign * MOVE_SPEED * math.cos(self.current_yaw)
                    vy = sign * MOVE_SPEED * math.sin(self.current_yaw)
                    self.pub_offboard(use_velocity=True)
                    self.pub_velocity(vx, vy, 0.0)
                    self.update_hold()

                elif cmd in ('a', 'd'):
                    yr = -YAW_RATE if cmd == 'a' else YAW_RATE
                    self.pub_offboard(use_velocity=True)
                    self.pub_velocity(0.0, 0.0, 0.0, yaw_rate=yr)
                    self.update_hold()

                elif cmd in ('r', 'f'):
                    vz = -VERT_SPEED if cmd == 'r' else VERT_SPEED  # NED: negative = up
                    self.pub_offboard(use_velocity=True)
                    self.pub_velocity(0.0, 0.0, vz)
                    self.update_hold()

                else:
                    self.pub_offboard(use_velocity=False)
                    self.pub_position(self.hold_x, self.hold_y, self.hold_z, yaw=self.hold_yaw)

            elif self.state == 'LANDING':
                self.pub_offboard()
                self.pub_position(self.current_x, self.current_y, HOVER_ALT)
                if self.counter % 40 == 0:
                    self.land()
                    self.get_logger().info(f'Landing... z={self.current_z:.2f}')

        except Exception as e:
            self.get_logger().error(f'EXCEPTION: {e}\n{traceback.format_exc()}')


def main():
    rclpy.init()
    node = ManualSurveyNode()
    rclpy.spin(node)
    node.running = False


if __name__ == '__main__':
    main()
