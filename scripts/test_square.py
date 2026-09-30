#!/usr/bin/env python3

import argparse
import math
import traceback
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)


class OffboardSquare(Node):

    def __init__(self, drone_ns, target_system):
        super().__init__(f'{drone_ns}_offboard_square')

        self.drone_ns = drone_ns
        self.target_system = target_system

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.get_logger().info('========================================')
        self.get_logger().info('INITIALIZING OFFBOARD SQUARE CONTROLLER')
        self.get_logger().info(f'DRONE NAMESPACE : {self.drone_ns}')
        self.get_logger().info(f'TARGET SYSID    : {self.target_system}')
        self.get_logger().info('========================================')

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.offboard_mode_topic = (
            f'/{self.drone_ns}/fmu/in/offboard_control_mode'
        )

        self.setpoint_topic = (
            f'/{self.drone_ns}/fmu/in/trajectory_setpoint'
        )

        self.command_topic = (
            f'/{self.drone_ns}/fmu/in/vehicle_command'
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            self.offboard_mode_topic,
            qos
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            self.setpoint_topic,
            qos
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            self.command_topic,
            qos
        )

        self.get_logger().info('Publishers initialized:')
        self.get_logger().info(f'  {self.offboard_mode_topic}')
        self.get_logger().info(f'  {self.setpoint_topic}')
        self.get_logger().info(f'  {self.command_topic}')

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.status_topic = (
            f'/{self.drone_ns}/fmu/out/vehicle_status_v4'
        )

        self.local_position_topic = (
            f'/{self.drone_ns}/fmu/out/vehicle_local_position_v1'
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            self.status_topic,
            self.status_callback,
            qos
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            self.local_position_topic,
            self.local_position_callback,
            qos
        )

        self.get_logger().info('Subscribers initialized:')
        self.get_logger().info(f'  {self.status_topic}')
        self.get_logger().info(f'  {self.local_position_topic}')

        # ---------------------------------------------------------
        # State Variables
        # ---------------------------------------------------------

        self.arming_state = 0
        self.nav_state = 0

        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.start_x = None
        self.start_y = None

        self.target_altitude = -5.0

        self.offboard_setpoint_counter = 0

        self.waypoint_index = 0
        self.waypoint_hold_counter = 0

        self.takeoff_hold_counter = 0
        self.takeoff_hold_time = 60  # 3 seconds @ 20Hz

        self.state = 'WAIT_FOR_POSITION'
        self.previous_state = None

        self.waypoints = []

        # ---------------------------------------------------------
        # Main Timer
        # ---------------------------------------------------------

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info('Main control loop started at 20Hz')

    # =============================================================
    # Callbacks
    # =============================================================

    def status_callback(self, msg):

        old_arm = self.arming_state
        old_nav = self.nav_state

        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

        if (
            old_arm != self.arming_state or
            old_nav != self.nav_state
        ):
            self.get_logger().info(
                f'STATUS UPDATE | '
                f'arming_state={self.arming_state} '
                f'nav_state={self.nav_state}'
            )

    def local_position_callback(self, msg):

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    # =============================================================
    # Utility Functions
    # =============================================================

    def timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def log_state_transition(self, new_state):

        if self.previous_state != new_state:

            self.get_logger().info(
                f'STATE TRANSITION | '
                f'{self.previous_state} --> {new_state}'
            )

            self.previous_state = new_state

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        msg.position = [
            float(x),
            float(y),
            float(z)
        ]

        msg.yaw = 0.0

        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):

        self.get_logger().info(
            f'SENDING COMMAND | '
            f'command={command} '
            f'param1={param1} '
            f'param2={param2}'
        )

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = command
        msg.param1 = param1
        msg.param2 = param2

        msg.target_system = self.target_system
        msg.target_component = 1

        msg.source_system = 255
        msg.source_component = 0

        msg.from_external = True

        self.command_pub.publish(msg)

    def arm(self):

        self.get_logger().info(
            'Attempting ARM command...'
        )

        # MAV_CMD_COMPONENT_ARM_DISARM
        self.send_command(
            400,
            param1=1.0,
            param2=21196.0
        )

    def set_offboard_mode(self):

        self.get_logger().info(
            'Attempting OFFBOARD mode switch...'
        )

        # MAV_CMD_DO_SET_MODE
        self.send_command(
            176,
            param1=1.0,
            param2=6.0
        )

    def land(self):

        self.get_logger().info(
            'Attempting LAND command...'
        )

        # MAV_CMD_NAV_LAND
        self.send_command(21)

    def distance_to_waypoint(self, waypoint):

        if (
            self.current_x is None or
            self.current_y is None or
            self.current_z is None
        ):
            return float('inf')

        dx = self.current_x - waypoint[0]
        dy = self.current_y - waypoint[1]
        dz = self.current_z - waypoint[2]

        return math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

    # =============================================================
    # Mission Setup
    # =============================================================

    def build_relative_waypoints(self):

        self.start_x = self.current_x
        self.start_y = self.current_y

        sx = self.start_x
        sy = self.start_y
        z = self.target_altitude

        self.get_logger().info(
            f'Captured start position | '
            f'x={sx:.2f} '
            f'y={sy:.2f} '
            f'z={self.current_z:.2f}'
        )

        self.waypoints = [
            (sx + 1.0, sy,       z),
            (sx + 1.0, sy + 1.0, z),
            (sx,       sy + 1.0, z),
            (sx,       sy,       z),
        ]

        self.get_logger().info('Generated mission waypoints:')

        for i, wp in enumerate(self.waypoints):

            self.get_logger().info(
                f'  WP{i + 1} | '
                f'x={wp[0]:.2f} '
                f'y={wp[1]:.2f} '
                f'z={wp[2]:.2f}'
            )

    # =============================================================
    # Main Control Loop
    # =============================================================

    def control_loop(self):

        try:

            self.log_state_transition(self.state)

            # Always stream heartbeat
            self.publish_offboard_mode()

            self.offboard_setpoint_counter += 1

            # -----------------------------------------------------
            # WAIT FOR POSITION
            # -----------------------------------------------------

            if self.state == 'WAIT_FOR_POSITION':

                if (
                    self.current_x is None or
                    self.current_y is None
                ):

                    if (
                        self.offboard_setpoint_counter % 40 == 0
                    ):
                        self.get_logger().warn(
                            'Waiting for local position estimate...'
                        )

                    return

                self.get_logger().info(
                    'Local position estimate acquired.'
                )

                self.build_relative_waypoints()

                self.state = 'PREFLIGHT'

                return

            current_wp = self.waypoints[self.waypoint_index]

            # Always stream active setpoint
            self.publish_setpoint(*current_wp)

            # -----------------------------------------------------
            # PREFLIGHT
            # -----------------------------------------------------

            if self.state == 'PREFLIGHT':

                if self.offboard_setpoint_counter % 20 == 0:

                    self.get_logger().info(
                        f'PREFLIGHT | '
                        f'streaming setpoints... '
                        f'count={self.offboard_setpoint_counter}'
                    )

                if self.offboard_setpoint_counter >= 40:

                    self.set_offboard_mode()

                    self.state = 'OFFBOARD'

            # -----------------------------------------------------
            # OFFBOARD
            # -----------------------------------------------------

            elif self.state == 'OFFBOARD':

                if self.offboard_setpoint_counter % 20 == 0:

                    self.get_logger().info(
                        'Waiting for OFFBOARD engagement...'
                    )

                if self.offboard_setpoint_counter >= 60:

                    self.arm()

                    self.state = 'ARMED'

            # -----------------------------------------------------
            # ARMED
            # -----------------------------------------------------

            elif self.state == 'ARMED':

                if self.arming_state == 2:

                    self.get_logger().info(
                        'Vehicle successfully armed.'
                    )

                    self.get_logger().info(
                        'Holding takeoff position before waypoint mission...'
                    )

                    self.takeoff_hold_counter = 0

                    self.state = 'TAKEOFF_HOLD'

                elif self.offboard_setpoint_counter % 20 == 0:

                    self.get_logger().warn(
                        'Vehicle not armed yet. Retrying arm command...'
                    )

                    self.arm()

            # -----------------------------------------------------
            # TAKEOFF HOLD
            # -----------------------------------------------------

            elif self.state == 'TAKEOFF_HOLD':

                self.takeoff_hold_counter += 1

                # Hold directly above starting location
                self.publish_setpoint(
                    self.start_x,
                    self.start_y,
                    self.target_altitude
                )

                if self.takeoff_hold_counter % 20 == 0:

                    self.get_logger().info(
                        f'TAKEOFF HOLD | '
                        f'current_z={self.current_z:.2f} '
                        f'target_z={self.target_altitude:.2f}'
                    )

                if self.takeoff_hold_counter >= self.takeoff_hold_time:

                    self.get_logger().info(
                        'Takeoff stabilization complete. '
                        'Beginning waypoint mission.'
                    )

                    self.state = 'FLYING'

            # -----------------------------------------------------
            # FLYING
            # -----------------------------------------------------

            elif self.state == 'FLYING':

                distance = self.distance_to_waypoint(current_wp)

                if self.offboard_setpoint_counter % 20 == 0:

                    self.get_logger().info(
                        f'FLYING | '
                        f'wp={self.waypoint_index + 1}/4 '
                        f'current=('
                        f'{self.current_x:.2f}, '
                        f'{self.current_y:.2f}, '
                        f'{self.current_z:.2f}) '
                        f'target=('
                        f'{current_wp[0]:.2f}, '
                        f'{current_wp[1]:.2f}, '
                        f'{current_wp[2]:.2f}) '
                        f'distance={distance:.2f}m'
                    )

                if distance < 0.35:

                    self.waypoint_hold_counter += 1

                else:

                    self.waypoint_hold_counter = 0

                if self.waypoint_hold_counter > 20:

                    self.get_logger().info(
                        f'Waypoint '
                        f'{self.waypoint_index + 1} reached.'
                    )

                    self.waypoint_hold_counter = 0

                    self.waypoint_index += 1

                    if self.waypoint_index >= len(self.waypoints):

                        self.waypoint_index = (
                            len(self.waypoints) - 1
                        )

                        self.state = 'DONE'

                    else:

                        next_wp = self.waypoints[
                            self.waypoint_index
                        ]

                        self.get_logger().info(
                            f'Moving to next waypoint | '
                            f'x={next_wp[0]:.2f} '
                            f'y={next_wp[1]:.2f} '
                            f'z={next_wp[2]:.2f}'
                        )

            # -----------------------------------------------------
            # DONE
            # -----------------------------------------------------

            elif self.state == 'DONE':

                self.publish_setpoint(
                    self.start_x,
                    self.start_y,
                    self.target_altitude
                )

                self.get_logger().info(
                    'Mission complete. '
                    'Landing at home position.'
                )

                self.land()

                self.state = 'LANDING'

            # -----------------------------------------------------
            # LANDING
            # -----------------------------------------------------

            elif self.state == 'LANDING':

                if self.offboard_setpoint_counter % 40 == 0:

                    self.get_logger().info(
                        f'LANDING | '
                        f'current_z={self.current_z:.2f}'
                    )

        except Exception as e:

            self.get_logger().error(
                f'EXCEPTION IN CONTROL LOOP: {str(e)}'
            )

            self.get_logger().error(
                traceback.format_exc()
            )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--drone',
        required=True,
        help='drone_0, drone_1, drone_2, drone_3'
    )

    parser.add_argument(
        '--sysid',
        required=True,
        type=int,
        help='PX4 MAV_SYS_ID'
    )

    args = parser.parse_args()

    print('========================================')
    print('STARTING TEST SQUARE MISSION')
    print(f'DRONE : {args.drone}')
    print(f'SYSID : {args.sysid}')
    print('========================================')

    rclpy.init()

    node = OffboardSquare(
        drone_ns=args.drone,
        target_system=args.sysid
    )

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().warn(
            'Keyboard interrupt received.'
        )

    finally:

        if rclpy.ok():

            node.get_logger().info(
                'Destroying ROS node...'
            )

            node.destroy_node()

            rclpy.shutdown()

        print('ROS shutdown complete.')


if __name__ == '__main__':
    main()
