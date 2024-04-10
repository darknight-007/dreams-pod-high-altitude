import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import time

class PolarVectorPublisher(Node):
    def __init__(self):
        super().__init__('polar_vector_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'polar_vector', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.x = 0.0  # Angle in degrees
        self.y = -60.0  # Initial value for y
        self.delta_x = 5.0  # Increment for x in degrees
        self.delta_y = 10.0  # Increment for y

    def timer_callback(self):
        msg = Vector3()
        msg.x = self.x
        msg.y = self.y
        msg.z = 0.0  # Unused in this example

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

        # Update x and y for the next callback
        self.x += self.delta_x
        if self.x > 360:
            self.x = 0.0
            self.y += self.delta_y
            if self.y > 90:
                self.y = -10.0

def main(args=None):
    rclpy.init(args=args)
    polar_vector_publisher = PolarVectorPublisher()

    try:
        rclpy.spin(polar_vector_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        polar_vector_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
