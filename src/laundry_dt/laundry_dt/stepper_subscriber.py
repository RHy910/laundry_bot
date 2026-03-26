import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class StepperSubcriber(Node):
    def __init__(self):
        super().__init__('stepper_subscriber')

        self.positions = ["front","back"]

        self.subscriber = {
            pos: self.create_subscription(Float32,f'stepper/{pos}/cmd', lambda msg, p = pos: self.cmd_callback(msg,p),10)
            for pos in self.positions
        }
        
        self.get_logger().info("stepper subscriber started")
    
    def cmd_callback(self, msg,p):
        self.get_logger().info(f"{p}stepper [{msg.data}] steps")

def main():
    rclpy.init()
    node =StepperSubcriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
