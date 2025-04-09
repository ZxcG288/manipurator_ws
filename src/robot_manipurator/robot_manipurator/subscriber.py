import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class SubscriberNodeClass(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 15
        # ใช้ create_subscription เพื่อลงทะเบียน subscriber
        self.publisher = self.create_subscription(Image, self.topicNameFrames, self.listener_callback, self.queueSize)

    def listener_callback(self, imageMessage):
        self.get_logger().info("The image frame is received")
        # แปลงจาก ROS Image ไป OpenCV Image
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage) 
        # แสดงผลภาพจาก OpenCV
        cv2.imshow("camera video", openCVImage)
        cv2.waitKey(1)
  
def main(args = None):
    rclpy.init(args=args)
    node = SubscriberNodeClass()
    rclpy.spin(node)  # รอให้มีการรับข้อความจาก topic
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
