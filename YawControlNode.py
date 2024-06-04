import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget, MountControl
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from std_msgs.msg import Header
import math
from rclpy.clock import ROSClock
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

class YawControlNode(Node):
    def __init__(self):
        super().__init__('yaw_control')


        qos_profile = QoSProfile(reliability = ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.VOLATILE,
                history = HistoryPolicy.KEEP_LAST,
                depth = 1
                )
        self.actuator_control_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        self.publisher = self.create_publisher(MountControl, '/mavros/mount_control/command', qos_profile)

        self.subscription = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos_profile_sensor_data)
        self.current_orientation = None
        self.attitude_setpoint = AttitudeTarget()
        self.attitude_setpoint.type_mask = 3
        self.des_yaw = math.radians(0.0)  # Desired yaw angle in radians
        self.des_thrust = 0.1
        
        self.yaw_rate = 0.0
        self.yaw = 0.0
        self.K_p = 4.0  # Proportional gain
        self.K_d = 0
        self.create_timer(0.1, self.publish_command)
    

    def publish_command(self):
        command = MountControl()
        now = self.get_clock().now()
        command.header = Header()
        command.header.stamp = now.to_msg()
        command.mode = 2
        self.publisher.publish(command)
    
        diff_yaw = self.des_yaw - self.yaw
        self.attitude_setpoint.body_rate.z = diff_yaw
        diff_yaw_thrust = math.fabs(diff_yaw/math.pi)
        self.des_thrust = self.K_p*diff_yaw_thrust + self.K_d*math.fabs(self.yaw_rate)
        self.IGNORE_ROLL_RATE = 1
        self.IGNORE_PITCH_RATE = 2

        self.attitude_setpoint.header= Header()
        self.attitude_setpoint.header.stamp = now.to_msg()
        self.attitude_setpoint.thrust = self.des_thrust # This should be publishing the desired thrust to the motors but isn't
        self.actuator_control_pub.publish(self.attitude_setpoint)
        print(diff_yaw, diff_yaw_thrust, self.des_thrust)

    def imu_callback(self, msg):
        orientation_q = msg.orientation
#        euler = self.quaternion_to_euler(orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w)

        euler = orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w
        self.yaw = euler[2]
        self.yaw_rate = msg.angular_velocity.z



    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    yaw_control_node = YawControlNode()
    rclpy.spin(yaw_control_node)
    yaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
