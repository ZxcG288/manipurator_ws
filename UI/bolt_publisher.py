#! /usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from yolov8_msgs.msg import Yolov8Inference
from cv_bridge import CvBridge
class GUI(Node):

    def __init__(self):
        super().__init__('bolt_detector')

        self.bridge = CvBridge()
        self.target_point = [0, 0, 0]
        self.fx = 253.93635749816895
        self.fy = 253.93635749816895
        self.cx = 320
        self.cy = 240
        self.z = 0.7
        self.init_x = 0.2
        self.init_y = 0.6
        self.published = False  # ตัวแปรสำหรับเช็คว่ามีการ publish แล้วหรือยัง

        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/target_point',
            10
        )

    def yolo_callback(self, data):
        if self.published:
            return  # ถ้าเคย publish แล้ว ไม่ต้องทำอะไรอีก

        for r in data.yolov8_inference:
            points = np.array(r.coordinates).astype(np.int32).reshape([4, 2]) #For calculating the middle point of the bolt (bounding box)
            middle_point = np.sum(points, 0) / 4 #For calculating the middle point of the bolt (bounding box)

            # คำนวณตำแหน่ง bolt
            self.target_point[0] = -self.z * (middle_point[1] - self.cy) / self.fy + self.init_x
            self.target_point[1] = -self.z * (middle_point[0] - self.cx) / self.fx + self.init_y

            # คำนวณมุมการวาง
            dist1 = np.linalg.norm(points[0] - points[1])
            dist2 = np.linalg.norm(points[1] - points[2])
            if dist1 > dist2:
                denominator = points[0][0] - points[1][0]
                angle = math.atan2(points[0][1] - points[1][1], denominator) if denominator != 0 else math.pi/2
            else:
                denominator = points[1][0] - points[2][0]
                angle = math.atan2(points[1][1] - points[2][1], denominator) if denominator != 0 else math.pi/2

            self.target_point[2] = math.pi/2 - angle

            # Publish ค่า bolt
            target_point_msg = Float64MultiArray(data=self.target_point)
            self.publisher.publish(target_point_msg)
            self.published = True  # ตั้งค่าให้ publish แค่ครั้งเดียว
            self.get_logger().info(f"Published target point: {self.target_point}")

def main(args=None):
    rclpy.init(args=args)
    node = GUI()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
