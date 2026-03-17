import rclpy

from rclpy.node import Node # capital 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class DcMotorController(Node):
    def __init__(self):

        super().__init__('dc_motor_controller')
        self.positions = ["front","mid","back"]
        
        self.ultrasonic_subscription ={ 
            self.create_subscription(Float32,f'ultrasonic/{pos}/distance',lambda msg, p=pos: self.ultrasonic_callback(msg,p) ,10)
            for pos in self.positions
            }
        
        self.cmd_publisher = {
            pos: self.create_publisher(Twist, f'/{pos}/cmd_vel', 10)
            for pos in self.positions
            } 
    
    def ultrasonic_callback(self,msg,p):
        cmd = Twist() #parenthesis
        if msg.data < 10:
            cmd.linear.x = .0
            self.get_logger().info(f"[{p}] Motor Stop")
        else:
            cmd.linear.x = .5
            
        
        self.cmd_publisher[p].publish(cmd)
    

def main():
    rclpy.init()
    node = DcMotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
    

