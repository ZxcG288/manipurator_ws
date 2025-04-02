import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            CompressedImage,
            'image/compressed',
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        self.publisher = self.create_publisher(
            Image,
            'lane_detected',
            10)
        
    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.resize(frame, (640, 480))

            # Perspective transformation
            tl, bl, tr, br = (222, 387), (70, 472), (400, 380), (538, 472)
            pts1 = np.float32([tl, bl, tr, br])
            pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
            matrix = cv2.getPerspectiveTransform(pts1, pts2)
            transformed_frame = cv2.warpPerspective(frame, matrix, (640, 480))

            # Convert to HSV and threshold
            hsv = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)
            lower = np.array([0, 0, 200])
            upper = np.array([255, 50, 255])
            mask = cv2.inRange(hsv, lower, upper)

            # Find lane lines
            histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
            midpoint = histogram.shape[0] // 2
            left_base = np.argmax(histogram[:midpoint])
            right_base = np.argmax(histogram[midpoint:]) + midpoint
            
            # Draw lanes
            result = cv2.addWeighted(frame, 1, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 0.5, 0)
            cv2.line(result, (left_base, 480), (left_base, 0), (255, 0, 0), 2)
            cv2.line(result, (right_base, 480), (right_base, 0), (255, 0, 0), 2)
            
            # Show processed image
            cv2.imshow('Lane Detection', result)
            cv2.waitKey(1)
            
            # Publish processed image
            output_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
            self.publisher.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()