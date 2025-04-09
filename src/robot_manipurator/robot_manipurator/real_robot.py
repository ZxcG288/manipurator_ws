#!/usr/bin/env python3

from ultralytics import YOLO
import cv2
import os
import subprocess
import rclpy
import signal
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from interface_manipurator.msg import SpeedAngle
from std_msgs.msg import String

from cv_bridge import CvBridge
from yolov8_msgs.msg import InferenceResult, Yolov8Inference

class RealRobot(Node):
    def __init__(self):
        super().__init__("real_robot")
        self.status_message = ""
        self.stop_time = None  # Initialize stop_time attribute to None
        model_path = os.path.join(os.environ['HOME'], 'trained_model', 'sign.pt')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.yolov8_inference = Yolov8Inference()

        self.status_sub =self.create_subscription(
            String,
            "/manipurator_information",
            self.status_callback,
            10
        )
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            'image/compressed',
            self.camera_callback,
            10
        )

        self.car_publisher = self.create_publisher(
            SpeedAngle, 
            "/SpeedAngle", 
            10
        )
        self.car_timer = self.create_timer(0.01, self.car_publish)


        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        
        #this for align the sign and self
        self.launch_process = None
        self.action_mapping = {
            "parking_sign": self.trigger_parking_sign_action,
            "place_sign": self.trigger_place_sign_action,
            # "red_light": self.trigger_red_light_action,
            # "green_light": self.trigger_green_light_action,
            # "crosswalk_sign": self.trigger_crosswalk_sign_action
        }
    def status_callback(self, msg):
        self.status_message = msg.data

    def camera_callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            results = self.model(img, conf=0.70, verbose=False)

            self.yolov8_inference.header.frame_id = "inference"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()
            detected_classes = []

            for r in results:
                if hasattr(r, 'obb') and r.obb is not None and len(r.obb) > 0:
                    for box in r.obb:
                        inference_result = InferenceResult()
                        if hasattr(box, 'xyxyxyxy'):
                            b = box.xyxyxyxy[0].cpu().numpy().copy()
                            class_name = self.model.names[int(box.cls)]
                            inference_result.class_name = class_name
                            inference_result.coordinates = b.reshape(1, 8).tolist()[0]

                            detected_classes.append(class_name)

                            #for detect the signs
                            if class_name in self.action_mapping:
                                self.get_logger().info(f"{class_name.replace('_', ' ').capitalize()} detected!")
                                self.action_mapping[class_name]()
                            
                            self.yolov8_inference.yolov8_inference.append(inference_result)

            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

            annotated_frame = results[0].plot()
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.img_pub.publish(img_msg)

            # Update the detected sign for use in car_publish and compute_drive_command
            self.detected_classes = detected_classes
            

        except Exception as e:
            self.get_logger().error(f"Error in camera_callback: {str(e)}")

    def car_publish(self):
        msg = SpeedAngle()
        msg.speed, msg.angle = self.compute_drive_command()
        self.car_publisher.publish(msg)
        self.get_logger().info(f'Published: speed={msg.speed}, angle={msg.angle}')

    def compute_drive_command(self):
        speed = 10
        angle = 0
    # Detecting other signs (e.g., red light, green light, crosswalk)
        if hasattr(self, 'detected_classes'):
            if "red_light" in self.detected_classes:
                speed = 0
            elif "green_light" in self.detected_classes:
                speed = 10
            elif "crosswalk_sign" in self.detected_classes:
                speed = 5
        return speed, angle



    
    # This function will be called when the parking sign is detected
    def trigger_parking_sign_action(self):
        time.sleep(2)
        self.launch_and_terminate('mtc_pick.launch.py')
    
    def trigger_place_sign_action(self):
        time.sleep(2)
        self.launch_and_terminate('mtc_place.launch.py')

    # For start the launch file when it detect the parking and place sign
    def launch_and_terminate(self, launch_file):
        try:
            self.launch_process = subprocess.Popen(['ros2', 'launch', 'mtc_config1', launch_file])
            self.get_logger().info(f"Launched {launch_file} successfully.")
            
            start_time = time.time()  # Recrod the time
            while self.status_message != "Task execution succeeded": # It will loop the program if self.status_message != "Task execution succeeded"
                if self.status_message != "The task has been stated" and time.time() - start_time > 10: #it will check that mtc_node_test and mtc_place has been stated yet if it not start in time it will reset the program
                    self.get_logger().warning("Task has not started within time. Restarting process.") 

                    self.launch_process.send_signal(signal.SIGINT) #kill the process
                    time.sleep(2)
                    self.launch_process = subprocess.Popen(['ros2', 'launch', 'mtc_config1', launch_file])

                    start_time = time.time()  #Reset the time
                    self.get_logger().info(f"Launched {launch_file} again.")
                
                self.get_logger().info("Waiting for process to complete...")
                time.sleep(1)

            if self.launch_process:
                self.launch_process.send_signal(signal.SIGINT)
                self.get_logger().info(f"Terminated {launch_file} process.")
        except Exception as e:
            self.get_logger().error(f"Failed to launch {launch_file}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RealRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# def trigger_red_light_action(self):
    #     self.get_logger().info("Triggering action for red light!")
    #     self.get_logger().info("Red light detected! Stopping the car.")
    #     msg = SpeedAngle()
    #     msg.speed = 0  #stop the car
    #     msg.angle = 0
    #     self.car_publishers.publish(msg)
    
    # def trigger_green_light_action(self):
    #     self.get_logger().info("Triggering action for green light!")
    
    # def trigger_crosswalk_sign_action(self):
    #     self.get_logger().info("Triggering action for crosswalk sign!")