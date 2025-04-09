import rclpy 
from rclpy.node import Node
from std_msgs.msg import String

class talkerMain(Node):
    
    def __init__(self):
        super().__init__('talker')
        self.talker = self.create_publisher(String, '/chatter',10)
        self.get_logger().info("talker has been started")
        self.timer = self.create_timer(0, self.talker_callback)
    
    def talker_callback(self):
        msg = String()
        msg.data = str(3)
        self.talker.publish(msg)



def main(args = None):
    rclpy.init(args = args)
    node = talkerMain()
    rclpy.spin(node) 
    rclpy.shutdown()