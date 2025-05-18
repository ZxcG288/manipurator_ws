import rclpy 
from rclpy.node import Node
from std_msgs.msg import String

class talkerMain(Node):
    
    def __init__(self):
        super().__init__("talker")
        self.talker = self.create_subscription(String, "/web_topic", self.lis_callback, 10)
        self.get_logger().info("listener has been started")
        
    
    def lis_callback(self, msg):
        self.get_logger().info("I heard '%s'"%msg.data)


def main(args = None):
    rclpy.init(args = args)
    node = talkerMain()
    rclpy.spin(node)
    rclpy.shutdown()