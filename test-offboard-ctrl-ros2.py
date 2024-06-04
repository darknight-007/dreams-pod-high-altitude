import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget, ADSBVehicle
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped  # Import for pose data
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, radians
import cv2
from cv_bridge import CvBridge
from datetime import datetime, timezone
from pyorbital import astronomy
from pyorbital import planets
import time
import pyproj


class SubmarineAttitudeControl(Node):
    def __init__(self):
        super().__init__("att_ctrller")
        self.adsb_sub = self.create_subscription(ADSBVehicle, '/mavros_hub/adsb/vehicle', self.adsb_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros_hub/global_position/raw/fix', self.gps_callback, 10)
        self.br = CvBridge()
        self.actuator_control_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.image_subscriber = self.create_subscription(Image, '/flir_camera/image_raw', self.image_callback, 5)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)  # Subscribe to the pose topic
        self.detection_subscriber = self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.image_pub = self.create_publisher(Image, '/edited_image', 5)

        self.current_image = None
        self.current_orientation = None  # Field to store current orientation
        self.attitude_setpoint = AttitudeTarget()
        self.attitude_setpoint.type_mask = 7
        self.des_pitch = radians(0)
        self.des_yaw = None
        self.des_thrust = 0.1
        # south tempe
        self.latitude = 33.37557039913588
        self.longitude = -111.91379195037831
        self.airplane_lon = 0.0
        self.airplane_lat = 0.0
        self.probe_lon = 0.0
        self.probe_lat = 0.0
        self.fwd_azimuth = 0.0
        self.back_azimuth = 0

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def image_callback(self, msg):
        self.current_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        current_time = datetime.utcnow()
        sun_zenith_angle = astronomy.sun_zenith_angle(current_time, self.longitude, self.latitude)
        sun_altitude, sun_azimuth = astronomy.get_alt_az(current_time, self.longitude, self.latitude)
        moon = planets.Moon(current_time)
        rasc, decl, alt, azi = moon.topocentric_position(self.longitude, self.latitude)
        #print(alt, azi, sun_zenith_angle,sun_azimuth,sun_altitude)


    def detection_callback(self, msg):
        if self.current_orientation:
            r,p,y = euler_from_quaternion(self.current_orientation)
            #self.des_yaw = radians(y)

        if self.current_image is not None:
            for detection in msg.detections:
                # Draw a circle at the center of each detection
                center = (int(detection.centre.x), int(detection.centre.y))
                error = (640.0-detection.centre.x)/640.0
                Ki=-3
                if self.des_yaw is not None:
                    self.des_yaw = self.des_yaw + radians(Ki*error)
                    print(error, degrees(self.des_yaw))

                cv2.circle(self.current_image, center, 10, (0, 255, 0), -1)

            # Convert the image back to ROS message and publish
            edited_image_msg = self.br.cv2_to_imgmsg(self.current_image, "bgr8")
            self.image_pub.publish(edited_image_msg)
    
    def pose_callback(self, msg):
        self.current_orientation = msg.pose.orientation  # Extract the orientation quaternion
        r,p,y = euler_from_quaternion(self.current_orientation)
    
    def adsb_callback(self, data):
        self.airplane_lat = data.latitude
        self.airplane_lon = data.longitude
        geodesic = pyproj.Geod(ellps='WGS84')
        self.fwd_azimuth, self.back_azimuth, distance = geodesic.inv(self.probe_lon, self.probe_lat, self.airplane_lon, self.airplane_lat)
        self.des_yaw = radians(-(self.fwd_azimuth-90))
        print(self.airplane_lat, self.airplane_lon, self.des_yaw)

    def gps_callback(self, data):
        self.probe_lat = data.latitude
        self.probe_lon = data.longitude


    def timer_callback(self):
        if self.des_yaw is not None:
            self.des_quaternion = quaternion_from_euler(0, 0, self.des_yaw)
            self.attitude_setpoint.orientation.x = self.des_quaternion[0]
            self.attitude_setpoint.orientation.y = self.des_quaternion[1]
            self.attitude_setpoint.orientation.z = self.des_quaternion[2]
            self.attitude_setpoint.orientation.w = self.des_quaternion[3]
            self.attitude_setpoint.thrust = self.des_thrust
            self.actuator_control_pub.publish(self.attitude_setpoint)

def main(args=None):
    rclpy.init(args=args)
    submarine_attitude_control = SubmarineAttitudeControl()
    rclpy.spin(submarine_attitude_control)
    submarine_attitude_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

