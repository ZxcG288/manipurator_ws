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
from std_msgs.msg import String

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO(os.environ['HOME'] + '/yolov8obb_training/best.pt')

        self.yolov8_inference = Yolov8Inference()
        self.status_message = ""
        self.task_succeeded = False  # <-- เพิ่ม flag

        self.subscription = self.create_subscription(
            CompressedImage,
            'image/compressed',
            self.camera_callback,
            10)

        self.yolov8_pub = self.create_publisher(
            Yolov8Inference,
            "/Yolov8_Inference",
            1)
        
        self.img_pub = self.create_publisher(
            Image,
            "/inference_result",
            1)
        
        self.status_publisher = self.create_publisher(
            String,
            "/manipurator_information",
            10)
        
        self.status_subscriber = self.create_subscription(
            String,
            "/manipurator_information",
            self.status_callback,
            10)
        
        self.box_status_publisher = self.create_publisher(
            String,
            "/check_box_status",
            10)

        self.has_published_status = False
        self.box_detected = False  

    def status_callback(self, msg):
        self.status_message = msg.data
        if self.status_message == "Task execution succeeded":
            self.task_succeeded = True  # <-- ตั้ง flag ไว้เมื่อ task สำเร็จ

    def camera_callback(self, data):
        try:
            if not self.has_published_status:
                msg = String()
                msg.data = "The sub_yolo is running"
                self.status_publisher.publish(msg)
                self.has_published_status = True

            # Decode the compressed image data
            np_arr = np.frombuffer(data.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            results = self.model(img, conf=0.60, verbose=False)

            self.yolov8_inference.header.frame_id = "inference"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            self.box_detected = False  # reset ก่อน

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
                        self.box_detected = True
                else:
                    pass

            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame)
            self.img_pub.publish(img_msg)

            # ✅ เช็คเงื่อนไข: task สำเร็จ + เจอกล่อง
            if self.task_succeeded and self.box_detected:
                box_msg = String()
                box_msg.data = "BOX"
                self.box_status_publisher.publish(box_msg)
                self.get_logger().info("Published BOX after task success.")
                self.task_succeeded = False  # reset หลัง publish
                self.box_detected = False

        except Exception as e:
            self.get_logger().error(f"Error in camera_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
