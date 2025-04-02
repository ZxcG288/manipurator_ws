import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from cv_bridge import CvBridge


class PublisherNodeClass(Node):

    def __init__(self):
        super().__init__('publisher_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'camera2_ros/compressed_image'  # Updated topic name
        self.queueSize = 15
        self.publisher = self.create_publisher(CompressedImage, self.topicNameFrames, self.queueSize)
        self.periodCommunication = 0.0333  # ส่งที่ ~30 FPS
        self.timer = self.create_timer(self.periodCommunication, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        success, frame = self.camera.read()
        
        if success:
            # ปรับขนาดภาพให้เล็กลง (ลดขนาดเพื่อประหยัดแบนด์วิดท์)
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)

            # บีบอัดภาพเป็น JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # 50 = คุณภาพต่ำ, 100 = คุณภาพสูง
        try:
            _,buffer = cv2.imencode('.jpg', frame, encode_param)
            # สร้าง CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.format = "jpeg"
            compressed_msg.data = np.array(buffer).tobytes()

            self.publisher.publish(compressed_msg)
            self.get_logger().info(f"Publishing compressed image number {self.i}")
            self.i += 1
        except Exception as e:
            self.get_logger().error(f"Error during publishing: {str(e)}")
            


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNodeClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
