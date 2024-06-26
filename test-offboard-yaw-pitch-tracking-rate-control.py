import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget, MountControl
from sensor_msgs.msg import Imu, Image

from geometry_msgs.msg import PoseStamped  # Import for pose data
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, radians
import cv2
from cv_bridge import CvBridge
from datetime import datetime, timezone
from pyorbital import astronomy
from pyorbital import planets
from geometry_msgs.msg import Vector3
import time
import pyproj

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
import math
from rclpy.clock import ROSClock
from std_msgs.msg import Header


class PodAttitudeControl(Node):
    def __init__(self):
        super().__init__("att_ctrller")
        self.br = CvBridge()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Adjust the depth as needed
        )

        qos_profile_spot = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.actuator_control_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', qos_profile)
        
        self.subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile_sensor_data)

        self.subscription = self.create_subscription(
            Vector3,
            '/brightest_point_vector',
            self.spot_callback,
            qos_profile_spot)


        self.publisher = self.create_publisher(
            MountControl,
            '/mavros/mount_control/command',
            qos_profile_spot)

        self.current_orientation = None  # Field to store current orientation
        self.attitude_setpoint = AttitudeTarget()
        self.attitude_setpoint.type_mask = 3
        self.des_pitch = radians(0)
        self.des_yaw = math.radians(90.0)
        self.des_thrust = 0.3
        self.yaw_rate = 0.0
        self.spot_x_norm = 0.0
        self.spot_y_norm = 0.0
        self.timer = self.create_timer(0.1, self.publish_command)
        self.pitch = 0.0
        self.yaw = 0.0
        self.K_p = 1.0
        self.K_d = 0.0
        self.pitch_offset = 0.0

    def imu_callback(self, msg):
        # Assuming the orientation is represented as Quaternion
        # Convert Quaternion to Euler to get pitch
        orientation_q = msg.orientation
        euler = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        pitch_angle = euler[1]  # Index 1 for pitch in the returned tuple
        self.yaw = euler[2]
        self.yaw_rate = msg.angular_velocity.z
        self.pitch = math.degrees(pitch_angle + self.pitch_offset)

    def spot_callback(self, msg):
        self.spot_x_norm = msg.x
        self.spot_y_norm = msg.y 

    def publish_command(self):
        command = MountControl()
        now = self.get_clock().now()
        command.header = Header()
        command.header.stamp = now.to_msg()
        command.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        command.pitch = self.pitch
        self.publisher.publish(command)

        diff_yaw = self.K_p * (self.des_yaw - self.yaw)
        #diff_yaw = self.K_p*self.spot_x_norm
        #diff_yaw = -1.5
        self.attitude_setpoint.header = Header()
        self.attitude_setpoint.header.stamp = now.to_msg()
        self.attitude_setpoint.body_rate.z = diff_yaw
        # self.des_thrust = math.fabs(diff_yaw)/(4*math.pi)
        self.attitude_setpoint.thrust = self.des_thrust
        self.actuator_control_pub.publish(self.attitude_setpoint)
        print(self.spot_x_norm, self.yaw, diff_yaw, self.des_thrust)


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


def main(args=None):
    rclpy.init(args=args)
    pod_attitude_control = PodAttitudeControl()
    rclpy.spin(pod_attitude_control)
    pod_attitude_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
