import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import csv

class AttitudeBrightnessExtractor(Node):
    def __init__(self):
        super().__init__('attitude_brightness_extractor')
        self.subscription = self.create_subscription(
            Vector3,
            '/attitude_brightness_vector',
            self.listener_callback,
            10)
        self.csv_file = open('attitude_brightness_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.csv_writer.writerow(['Timestamp', 'Yaw', 'Pitch', 'Brightness'])  # Writing header of the CSV file

    def listener_callback(self, msg):
        timestamp = self.get_clock().now().to_msg()  # Getting the current timestamp
        self.csv_writer.writerow([timestamp.sec, msg.x, msg.y, msg.z])  # Writing data to CSV

    def __del__(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    extractor = AttitudeBrightnessExtractor()
    rclpy.spin(extractor)
    extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

