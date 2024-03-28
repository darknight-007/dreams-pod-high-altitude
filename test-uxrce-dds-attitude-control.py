#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleAttitudeSetpoint, VehicleCommand, VehicleAttitude, FailsafeFlags, \
    VehicleStatus
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from time import time 

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode using attitude setpoints."""

    def __init__(self) -> None:
        super().__init__('offboard_control_attitude')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        # Replace the vehicle status subscriber with a failsafe flags subscriber
        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.vehicle_status = VehicleStatus()

        # Initialize variables
        self.vehicle_attitude = VehicleAttitude()

        self.offboard_setpoint_counter = 0
        self.curr_pitch_deg = 0.0
        self.start_time = self.get_clock().now()
        self.pitch_target = 0.0
        self.cycle = 0.0
        self.TIME_SCALAR = 10

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.05, self.timer_callback)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_attitude topic subscriber."""
        self.vehicle_attitude = vehicle_attitude
        quat = self.vehicle_attitude.q
        curr_roll_rad, curr_pitch_rad, curr_yaw_rad = euler_from_quaternion(quat)
        self.curr_pitch_deg = math.degrees(curr_pitch_rad)

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_attitude_setpoint(self, roll: float, pitch: float, yaw: float, thrust: float):
        """Publish the attitude setpoint."""
        msg = VehicleAttitudeSetpoint()
        quat = quaternion_from_euler(0, 0, yaw)
        msg.q_d = quat 
        msg.yaw_body = 0.0
        msg.thrust_body = [0.0, 0.0, thrust]  # Assuming thrust is set in the z component
        clock__now = self.get_clock().now()
        nanoseconds = clock__now.nanoseconds
        msg.timestamp = int(nanoseconds / 1000)
        self.publish_offboard_control_heartbeat_signal()

        self.attitude_setpoint_publisher.publish(msg)
        self.get_logger().info(
            f"Publishing attitude setpoints: timestamp {self.cycle}, cosine {self.pitch_target}, Roll {roll}, Pitch {pitch}, Yaw {yaw}, Thrust {thrust}, Curr yaw (dg) {self.curr_pitch_deg}")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            target_yaw = math.cos(time()/2)/1.5
            self.publish_attitude_setpoint(0.0, 0.0, target_yaw, math.fabs(target_yaw))  # Example values

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node with attitude control...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
