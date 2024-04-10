#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self) -> None:
        super().__init__('image_processor')

        # Configure QoS profile for the image subscription and vector publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile)
        
        self.image_publisher = self.create_publisher(
            Image,
            '/image_processed',
            qos_profile)

        self.vector_publisher = self.create_publisher(
            Vector3,
            '/brightest_point_vector',
            qos_profile)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = current_frame.shape[:2]

        # Convert to grayscale and find the brightest pixel
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        (_, _, _, maxLoc) = cv2.minMaxLoc(gray)
        
        # Draw a green circle around the brightest pixel
        cv2.circle(current_frame, maxLoc, 20, (0, 255, 0), 2)

        # Convert the OpenCV image back to ROS Image message and publish it
        img_msg = self.bridge.cv2_to_imgmsg(current_frame, encoding="bgr8")
        self.image_publisher.publish(img_msg)

        # Calculate and publish the vector from the center to the brightest spot, normalized by image dimensions
        vector_msg = Vector3()
        vector_msg.x = (maxLoc[0] - width / 2) / width
        vector_msg.y = (maxLoc[1] - height / 2) / height
        vector_msg.z = 0.0  # Not used in this context
        self.vector_publisher.publish(vector_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
