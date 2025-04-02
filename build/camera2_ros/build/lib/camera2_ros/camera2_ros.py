import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from cv_bridge import CvBridge

class SubscriberNodeClass(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        self.bridgeObject = CvBridge()
        self.topicNameFrames = '/image_raw/compressed'
        self.queueSize = 15
        # ใช้ create_subscription เพื่อลงทะเบียน subscriber
        self.subscriber = self.create_subscription(
            CompressedImage,
            self.topicNameFrames,
            self.listener_callback,
            self.queueSize
        )

    def listener_callback(self, compressedImageMessage):
        self.get_logger().info("The compressed image frame is received")
        # แปลงจาก ROS2 CompressedImage ไป OpenCV Image
        np_arr = np.frombuffer(compressedImageMessage.data, np.uint8)  # แปลง buffer เป็น numpy array
        openCVImage = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # แปลงเป็น OpenCV image

        # แสดงผลภาพจาก OpenCV
        cv2.imshow("Camera Video", openCVImage)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNodeClass()
    rclpy.spin(node)  # รอให้มีการรับข้อความจาก topic
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
