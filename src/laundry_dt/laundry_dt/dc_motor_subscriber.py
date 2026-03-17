import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DcMotorSubcriber(Node):
    def __init__(self):
        super().__init__('dc_motor_subscriber')
        self.positions = ["front","mid","back"]#maybe call segments 

        self.subscribers = {
            pos: self.create_subscription(Twist,f'/{pos}/cmd_vel', lambda msg, p=pos: self.cmd_callback(msg, p) ,10)
            for pos in self.positions
            }
        
        self.get_logger().info("dc_motor subscribers started")
    
    def cmd_callback(self, msg,p):
        self.get_logger().info(f"{p} dc_motor Linear x: {msg.linear.x}, Angular z: {msg.angular.z}")

def main():
    rclpy.init()
    node =DcMotorSubcriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
