#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from px4_msgs.msg import ActuatorMotors, ActuatorServos, OffboardControlMode, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ActuatorControlPublisher(Node):
    def __init__(self):
        super().__init__('actuator_control_publisher')
        qos_profile_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        qos_profile_status = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Reliability: BEST_EFFORT
        # History(Depth): UNKNOWN
        # Durability: TRANSIENT_LOCAL
        # Lifespan: Infinite
        # Deadline: Infinite
        # Liveliness: AUTOMATIC
        # Liveliness
        # lease
        # duration: Infinite

        # Publisher for the motors
        self.motor_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile_commands)

        # Publisher for the servos
        self.servo_publisher = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', qos_profile_commands)

        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_commands)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile_status)

        # Timer to control the publishing rate at 10Hz
        self.vehicle_status = VehicleStatus()
        self.timer = self.create_timer(0.1, self.publish_actuator_commands)

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def publish_commands(self):
        # Publish the offboard control mode heartbeat signal
        self.publish_offboard_control_heartbeat_signal()

        # Publish actuator commands
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            self.publish_actuator_commands()

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode heartbeat."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_actuator_commands(self):
        # Motor command
        motor_command = ActuatorMotors()
        motor_command.control[0] = 0  # Example for motor 1
        motor_command.control[2] = 0  # Example for motor 3
        motor_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.motor_publisher.publish(motor_command)

        # # Servo command
        # servo_command = ActuatorServos()
        # servo_command.control[0] = 0.5  # Example for servo 1 (aux 1)
        # self.servo_publisher.publish(servo_command)

def main(args=None):
    rclpy.init(args=args)
    actuator_control_publisher = ActuatorControlPublisher()
    rclpy.spin(actuator_control_publisher)
    actuator_control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


