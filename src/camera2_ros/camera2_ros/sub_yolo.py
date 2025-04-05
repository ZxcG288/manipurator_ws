#!/usr/bin/env python3

from ultralytics import YOLO
import cv2
import os
import copy
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO(os.environ['HOME'] + '/yolov8obb_training/best.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            CompressedImage,
            'image/compressed',
            self.camera_callback,
            10)
        #self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):
        try:
            # Decode the compressed image data
            np_arr = np.frombuffer(data.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            results = self.model(img, conf=0.70, verbose=False)

            self.yolov8_inference.header.frame_id = "inference"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            for r in results:
                if r.obb is not None:
                    boxes = r.obb
                    for box in boxes:
                        self.inference_result = InferenceResult()
                        b = box.xyxyxyxy[0].to('cpu').detach().numpy().copy()
                        c = box.cls
                        self.inference_result.class_name = self.model.names[int(c)]
                        a = b.reshape(1, 8)
                        self.inference_result.coordinates = copy.copy(a[0].tolist())
                        self.yolov8_inference.yolov8_inference.append(self.inference_result)
                else:
                    pass
                    #self.get_logger().info(f"no_results")

            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame)
            self.img_pub.publish(img_msg)
        except Exception as e:
            pass
            #self.get_logger().error(f"Error in camera_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)  # รอให้มีการรับข้อความจาก topic
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
