import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher') # creates node untrasonic publisher from node class with super parent call
        self.publisher = self.create_publisher(Float32, 'ultrasonic/distance', 10)
        
        self.timer = self.create_timer(0.1, self.publish_reading)
        self.get_logger().info('Ultrasonic publisher started')

    def publish_reading(self):
        msg = Float32()
        msg.data = random.uniform(2.0, 400.0) # random for now ESP32 data later please rememeber 
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main():
    rclpy.init()
    node = UltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node(ultrasonic publisher)...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()