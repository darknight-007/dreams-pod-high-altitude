#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleAttitudeSetpoint, VehicleCommand, VehicleAttitude
import math
# Assuming FailsafeFlags exists and is the correct type for /fmu/out/failsafe_flags
from px4_msgs.msg import FailsafeFlags  # Adjust this import based on the actual message type


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
        self.failsafe_flags_subscriber = self.create_subscription(
            FailsafeFlags, '/fmu/out/failsafe_flags', self.failsafe_flags_callback, qos_profile)

        # Initialize variables
        self.vehicle_attitude = VehicleAttitude()
        # Replace vehicle status with failsafe flags variable
        self.failsafe_flags = FailsafeFlags()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_attitude topic subscriber."""
        self.vehicle_attitude = vehicle_attitude

    def failsafe_flags_callback(self, failsafe_flags):
        """Callback function for failsafe_flags topic subscriber."""
        self.failsafe_flags = failsafe_flags

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
        msg.roll_body = roll
        msg.pitch_body = pitch
        msg.yaw_body = yaw
        msg.thrust_body = [0.0, 0.0, thrust]  # Assuming thrust is set in the z component
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.attitude_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing attitude setpoints: Roll {roll}, Pitch {pitch}, Yaw {yaw}, Thrust {thrust}")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # Check for offboard mode using failsafe flags instead of vehicle status
        # You will need to adjust the condition below based on how offboard mode is represented in the failsafe flags
        if self.failsafe_flags.offboard_control_signal_lost:  # Adjust this condition based on your message structure
            print("Offboard control signal lost, adjust your conditions and actions accordingly")
        else:
            # Example of publishing an attitude setpoint. Adjust values as needed.
            current_yaw_quat = self.vehicle_attitude.q
            roll, pitch, current_yaw = self.quaternion_to_euler(current_yaw_quat[0], current_yaw_quat[1],
                                                                current_yaw_quat[2], current_yaw_quat[3])
            yaw_desired = 1.0
            yaw_delta = yaw_desired - current_yaw
            print(yaw_desired, current_yaw, yaw_delta)
            self.publish_attitude_setpoint(0.0, 0.0, yaw_desired, 0.6)  # Example values

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z  # in radians


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
