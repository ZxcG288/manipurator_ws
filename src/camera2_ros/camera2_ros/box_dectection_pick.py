#! /usr/bin/python3
# -*- coding: utf-8 -*-
#import sys
#import rclpy.destroyable

import math
import numpy as np
import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Image
from yolov8_msgs.msg import Yolov8Inference
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose  # ใช้ Pose แทน Float64MultiArray

class GUI(Node):

    def __init__(self):
        super().__init__('box_detector')

        self.bridge = CvBridge()
        self.fx = 760
        self.fy = 760
        self.cx = 320
        self.cy = 240
        self.z = -0.25
        self.init_x = -0.235
        self.init_y = 0.125
        self.published = False  # ตัวแปรสำหรับเช็คว่ามีการ publish แล้วหรือยัง

        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10
        )

        self.publisher = self.create_publisher(
            Pose,  # เปลี่ยนจาก Float64MultiArray เป็น Pose
            '/position_topic',
            10
        )

    def yolo_callback(self, data):
        if self.published:
            return  # ถ้าเคย publish แล้ว ไม่ต้องทำอะไรอีก

        for r in data.yolov8_inference:
            points = np.array(r.coordinates).astype(np.int32).reshape([4, 2]) # คำนวณจุดกึ่งกลาง
            middle_point = np.sum(points, 0) / 4  

            # คำนวณตำแหน่ง bolt
            target_x = -self.z * (middle_point[1] - self.cy) / self.fy + self.init_x
            target_y = -self.z * (middle_point[0] - self.cx) / self.fx + self.init_y
            target_z = 0.04  # ค่าคงที่ หรืออาจเปลี่ยนได้ตามต้องการ

            # คำนวณมุมการวาง
            dist1 = np.linalg.norm(points[0] - points[1])
            dist2 = np.linalg.norm(points[1] - points[2])
            if dist1 > dist2:
                denominator = points[0][0] - points[1][0]
                angle = math.atan2(points[0][1] - points[1][1], denominator) if denominator != 0 else math.pi/2
            else:
                denominator = points[1][0] - points[2][0]
                angle = math.atan2(points[1][1] - points[2][1], denominator) if denominator != 0 else math.pi/2

            yaw = math.pi/2 - angle  # คำนวณ yaw

            # สร้างข้อความ Pose
            pose_msg = Pose()
            pose_msg.position.x = target_x
            pose_msg.position.y = target_y
            pose_msg.position.z = target_z
            pose_msg.orientation.z = yaw  # แกน z ใช้แทน yaw

            # Publish ค่า bolt
            self.publisher.publish(pose_msg)
            self.published = False  # ตั้งค่าให้ publish แค่ครั้งเดียว
            #self.get_logger().info(f"Published position: x={target_x}, y={target_y}, z={target_z}, yaw={yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = GUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
