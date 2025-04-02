import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import os
import time

class NodeSwitcher(Node):
    def __init__(self):
        super().__init__('node_switcher')
        self.srv = self.create_service(SetBool, 'switch_nodes', self.switch_callback)
        self.get_logger().info("Node Switcher is ready. Use the service to switch nodes.")

    def switch_callback(self, request, response):
        # Stop both nodes before starting the selected one
        self.get_logger().info("Stopping all nodes...")
        os.system("pkill -f /home/g288-pc/project1_ws/src/install/server_controllers/lib/server_controllers/node1") # ปิด Node 1
        os.system("pkill -f /home/g288-pc/project1_ws/src/install/server_controllers/lib/server_controllers/node2") # ปิด Node 2
        time.sleep(0.2)
        
        if request.data:  # ถ้าได้รับคำสั่ง True ให้เปิด Node 2
            self.get_logger().info("Switching to Node 2...")
            os.system("ros2 run server_controllers node2 &")  # เปิด Node 2
            response.success = True
            response.message = "Switched to Node 2"
        else:  # ถ้าได้รับคำสั่ง False ให้เปิด Node 1
            self.get_logger().info("Switching to Node 1...")
            os.system("ros2 run server_controllers node1 &")  # เปิด Node 1
            response.success = True
            response.message = "Switched to Node 1"
        
        return response

def main():
    rclpy.init()
    node = NodeSwitcher()
    rclpy.spin(node)  # ทำให้โหนดทำงานไปเรื่อยๆ
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
