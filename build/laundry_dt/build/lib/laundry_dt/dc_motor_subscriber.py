import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DcMotorSubcriber(Node):
    def __init__(self):
        super().__init__('dc_motor_subscriber')
        self.subscriber = self.create_subscription(Twist,'/cmd_vel', self.cmd_callback,10)
        
        self.get_logger().info("dc_motor subscriber started")
    
    def cmd_callback(self, msg):
        self.get_logger().info(f"Linear x: {msg.linear.x}, Angular z: {msg.angular.z}")

def main():
    rclpy.init()
    node =DcMotorSubcriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
