#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint, VehicleCommand, VehicleAttitude, VehicleStatus


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode using attitude rate setpoints."""

    def __init__(self) -> None:
        super().__init__('offboard_control_rate')

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
        self.attitude_rate_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status = VehicleStatus()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.05, self.timer_callback)

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_attitude topic subscriber."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    # Methods for arm, disarm, engage_offboard_mode, land remain the same

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_attitude_rate_setpoint(self, roll_rate: float, pitch_rate: float, yaw_rate: float, thrust: float):
        """Publish the attitude rate setpoint."""
        msg = VehicleRatesSetpoint()
        msg.roll = roll_rate
        msg.pitch = pitch_rate
        msg.yaw = yaw_rate
        # Set thrust if applicable for your control strategy; it might need a different approach
        msg.thrust_body = [0.0, 0.0, thrust]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.attitude_rate_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing attitude rate setpoints: Roll rate {roll_rate}, Pitch rate {pitch_rate}, Yaw rate {yaw_rate}, Thrust {thrust}")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # Example of publishing an attitude rate setpoint. Adjust values as needed.
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_attitude_rate_setpoint(0.0, 0.0, 3,0.6)



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
