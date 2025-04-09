import rclpy
from rclpy.node import Node
from project_values.msg import Float32

class Project1Main(Node):
    
    def __init__(self):
        super().__init__("project1")
        self.pub = self.create_publisher(Float32, "/topic", 10)
        self.pub_timer = self.create_timer(1, self.pub_callback)
    
    def pub_callback(self):
        msg = Float32()
        msg.speed = 10.0
        msg.car_angle = 2.0
        msg.angle1 = 2.0
        msg.angle2 = 2.0
        msg.angle3 = 2.0
        msg.angle4 = 2.0
        msg.angle5 = 2.0
        self.pub.publish(msg)
        self.get_logger().info(f"The Data is : '{msg.car_angle}' '{msg.speed}'")

def main(args=None):
    rclpy.init(args=args)
    node = Project1Main()
    rclpy.spin(node)
    rclpy.shutdown()
