import rclpy

from rclpy.node import Node # capital 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class DcMotorController(Node):
    def __init__(self):
        super().__init__('dc_motor_controller')
        self.ultrasonic_subscription = self.create_subscription(Float32,'ultrasonic/distance', self.ultrasonic_callback ,10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10) #tyoe is twist
    
    def ultrasonic_callback(self,msg):
        cmd = Twist() #parenthesis
        if msg.data < 10:
            cmd.linear.x = .0
            self.get_logger().info(f"Motor Stop")
        else:
            cmd.linear.x = .5
            
        
        self.cmd_publisher.publish(cmd)
    

def main():
    rclpy.init()
    node = DcMotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
    

