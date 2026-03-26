import rclpy

from rclpy.node import Node # capital 
from std_msgs.msg import Float32



class StepperController(Node):
    def __init__(self):
        super().__init__('stepper_controller')
        self.positions = ["front","back"]
        self.stepper_subscriber = {}
    
        self.cmd_publisher = {}

        for pos in self.positions:
            self.stepper_subscriber[pos] = self.create_subscription(Float32,f'stepper/{pos}/cmd', lambda msg, p = pos: self.stepper_callback(msg,p) ,10)
            self.cmd_publisher[pos] = self.create_publisher(Float32, f'stepper/{pos}/steps', 10) 

        self.winch_subscriber = self.create_subscription(Float32, '/mid/winch/cmd', lambda msg: self.winch_callback(msg), 10)
        self.winch_publisher = self.create_publisher(Float32, '/mid/winch/steps', 10)

        self.get_logger().info("Stepper controller initialized")


    def stepper_callback(self,msg,p):
        self.cmd_publisher[p].publish(msg)
        
        self.get_logger().info(f'Recieved and Publishing {p} stepper {msg.data} steps')
    
    def winch_callback(self,msg):
        self.winch_publisher.publish(msg)
        self.get_logger().info(f'Recieved and Publishing mid winch {msg.data} steps')
        

    

def main():
    rclpy.init()
    node = StepperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    