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
from std_msgs.msg import String, Float32
from tensorflow.keras.models import load_model
from collections import deque

from cv_bridge import CvBridge
from yolov8_msgs.msg import InferenceResult, Yolov8Inference

class RealRobot(Node):
    def __init__(self):
        super().__init__("real_robot")
        model_path = os.path.join(os.environ['HOME'], 'trained_model', 'sign.pt')
        self.model = YOLO(model_path)

        #   <-- Self driving model -->
        self_driving_model_path = os.path.join(os.environ['HOME'], 'neural_network_self_driving_ws/keras', 'advance_33.keras')
        self.self_driving_model = load_model(self_driving_model_path, compile=False)
        #   <-- Self driving model -->
        
        self.bridge = CvBridge()
        self.yolov8_inference = Yolov8Inference()

        # Subscribe to the manipulator status topic for update status pick and place 
        self.status_sub =self.create_subscription(
            String,
            "/manipurator_information",
            self.status_callback,
            10
        )

        # Subscribe to the camera topic
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            'image/compressed',
            self.camera_callback,
            10
        )

        # Publishe to Plot grahp
        self.plot_steering_angle_publisher = self.create_publisher(
            Float32,
            '/steering_angle',
            10,
        )

        # Publisher to the car control topic for speed and angle control
        self.car_publisher = self.create_publisher(
            SpeedAngle, 
            "/SpeedAngle", 
            10
        )
        
        # Subscribe to check the status of the box
        self.box_status_subscriber = self.create_subscription(
            String,
            "/check_box_status",
            self.box_status_callback,
            10
        )
        
        # Publisher to display detection results in RVIZ2
        self.yolov8_pub = self.create_publisher(
            Yolov8Inference,
            "/Yolov8_Inference", 
            1
        )
        
        # Publisher to display YOLOv8 detection results
        self.img_pub = self.create_publisher(
            Image, 
            "/inference_result", 
            1
        )
        
        # This for align the sign and self to tiggle the action in def launch_subprocess(self, launch_file):
        self.launch_process = None
        self.action_mapping = {
            "parking_sign": self.trigger_parking_sign_action,
            "place_sign": self.trigger_place_sign_action,
            # "red_light": self.trigger_red_light_action,
            # "green_light": self.trigger_green_light_action,
            # "crosswalk_sign": self.trigger_crosswalk_sign_action
        }
        self.triggered_actions = set()
        
        # Set up the values
        self.status_message = ""
        self.box_status = ""
        self.current_sign_status = "red"
        self.pick_ditected = False
        self.place_ditected = False
        
        # Check time of the launch file and cross_walk sign timer
        self.sub_yolo_time = None
        self.task_execution_succeeded_time = None
        self.time_to_ready_to_go = None #Timer for mtc_ready_to_go.launch.py
        self.crosswalk_detected_time = None #For crosswalk sign
        
        # Check thhe status of terms that Check if it has been used before
        self.stop_and_go = False
        self.launching = False 
        self.task_completed_once = False
        self.The_manipurator_ready_to_go = False
        self.ready_to_go = False


        # self.timer_callback is for check the status of the launch file of the manipulator pick and place
        self.timer = self.create_timer(0.5, self.timer_callback)

        # self.car_publish for car control
        self.car_timer = self.create_timer(0.1, self.car_publish)

        # self.plot_steering_angle_publisher for Ploting Grahp
        self.plot_steering = self.create_timer(0.067, self.plot_steering_angle)

        # Imgage for self driving model
        self.self_driving_img = None
        self.steering = 0.0
        self.angle = 90.0 # 90 degree mean straight direction
        self.last_smoothed_angle = 90.0

        #### For LSTM self driving model
        # self.seq_len = 5
        # self.seq_buffer = deque(maxlen=self.seq_len)
    
    def denormalize_clamped(self, x, new_min=-1.0, new_max=1.0, old_min=45, old_max=135):
        # Clamp input to range -1 to 1
        x = max(min(x, new_max), new_min)
        denormalized = ((x - new_min) / (new_max - new_min) * (old_max - old_min) + old_min)
        return denormalized

    #Preprocess the image for self driving model
    def preprocess(self, img):
        img = img[50:, :]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        img = cv2.resize(img, (200, 66))
        img = img / 255.0
        return img
 
    def camera_callback(self, data):
        if not self.stop_and_go: #to stop the camera callback running when the launch file is running 
            try:
                np_arr = np.frombuffer(data.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                results = self.model(img, conf=0.9, verbose=False) #Sign prediction at 90% confidence above
                
                #   <-- Self driving model -->
                self.self_driving_img = cv2.resize(img, (240, 120))
                self.self_driving_img = self.preprocess(self.self_driving_img)
                self.self_driving_img = np.array([self.self_driving_img])
                self.steering = float(self.self_driving_model.predict(self.self_driving_img)[0][0]) * 1.1
                alpha = 1.0
                smoothed_angle = alpha * self.steering + (1 - alpha) * self.last_smoothed_angle
                self.last_smoothed_angle = smoothed_angle
                self.angle = self.denormalize_clamped(smoothed_angle)    
                self.get_logger().info(f"Steering angle: {self.angle}")
                # self.self_driving_img = self.preProcess(img)
                # streering = float(self.self_driving_model.predict(self.self_driving_img))
                # streering = float(self.self_driving_model.predict(self.self_driving_img)[0][0]) * 1.1
                # alpha = 1.0
                # smoothed_angle = alpha * streering + (1 - alpha) * self.last_smoothed_angle
                # self.last_smoothed_angle = smoothed_angle
                # self.angle = self.denormalize_clamped(smoothed_angle)
                # self.angle = self.denormalize_clamped(streering) 
                # self.get_logger().info(f"Input shape: {self.steering}")
                #   <-- Self driving model -->
                
                #   <-- Sign detection -->
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

                                # For detect the signs to car control topic
                                if class_name == "green_light":
                                    self.current_sign_status = "green"
                                    self.get_logger().info("Green light detected!")
                                elif class_name == "red_light":
                                    self.current_sign_status = "red"
                                    self.get_logger().info("Red light detected!")
                                elif class_name == "crosswalk_sign":    
                                    self.current_sign_status = "cross_walk"
                                    self.crosswalk_detected_time = time.time() # Set the time of the crosswalk sign
                                    self.get_logger().info("Crosswalk sign detected!")
                                elif class_name == "parking_sign":
                                    if not self.pick_ditected:
                                        self.current_sign_status = "parking"
                                        self.triggered_actions.discard("place_sign")  # Reset the place sign action
                                        self.get_logger().info("Parking sign detected!")
                                        self.pick_ditected = True
                                        self.place_ditected = False
                                elif class_name == "place_sign":
                                    if not self.place_ditected:
                                        self.current_sign_status = "place"
                                        self.triggered_actions.discard("parking_sign") # Reset the parking sign action
                                        self.get_logger().info("Place sign detected!")
                                        self.place_ditected = True
                                        self.pick_ditected = False

                                # Trigger action only once per detected sign
                                if class_name in self.action_mapping and class_name not in self.triggered_actions:
                                    self.get_logger().info(f"{class_name.replace('_', ' ').capitalize()} detected!")
                                    self.action_mapping[class_name]()  # Trigger the mapped action
                                    self.triggered_actions.add(class_name)  # Mark as triggered
                                
                                
                                self.yolov8_inference.yolov8_inference.append(inference_result)

                self.yolov8_pub.publish(self.yolov8_inference)
                self.yolov8_inference.yolov8_inference.clear()

                annotated_frame = results[0].plot()
                img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                self.img_pub.publish(img_msg)

                # Update the detected sign for use in def car_publish
                self.detected_classes = detected_classes
                #   <-- Sign detection -->
                

            except Exception as e:
                self.get_logger().error(f"Error in camera_callback: {str(e)}")
    
    # To Ploting Grahp
    def plot_steering_angle(self):
        pass
        # msg = Float32()
        # msg.data = self.steering
        # self.plot_steering_angle_publisher.publish(msg)
        # self.get_logger().info(f"Plot Value: {msg.data}")
        
    def car_publish(self):
        msg = SpeedAngle()
        msg.speed = 0
        # if self.current_sign_status == "green" or self.current_sign_status == "cross_walk":
        #     msg.angle = int(self.angle)# msg.angle = 90#
        #     msg.speed = 50 
        if self.current_sign_status == "green":
            msg.speed = 50
            msg.angle = int(self.angle)
        elif self.current_sign_status == "red":
            msg.speed = 0
            msg.angle = 90
        elif self.current_sign_status == "cross_walk":
            if self.crosswalk_detected_time is not None:
                elapsed_time_of_crosswalk_detected = time.time() - self.crosswalk_detected_time
                if elapsed_time_of_crosswalk_detected < 5.0:
                    msg.angle = int(self.angle)
                    msg.speed = 35
                else: 
                    self.current_sign_status = "green"
                    self.crosswalk_detected_time = None
        elif self.current_sign_status == "parking":
            msg.speed = 0
            msg.angle = 90
        elif self.current_sign_status == "place":
            msg.speed = 0
            msg.angle = 90
        else:
            msg.speed = 0
            msg.angle = 90
        self.car_publisher.publish(msg)
        #self.get_logger().info(f'Published: speed={msg.speed}, angle={msg.angle}')

    def box_status_callback(self, msg): 
        self.box_status = msg.data
        
    def status_callback(self, msg):
        self.status_message = msg.data
        self.get_logger().info(f"Status message: {self.status_message}")

        if self.status_message == "The sub_yolo is running" and self.sub_yolo_time is None:
            self.sub_yolo_time = time.time()
            self.get_logger().info("Start time initialized.")

        elif self.status_message == "The task has been stated":
            self.get_logger().info("Task has been started.")
            self.sub_yolo_time = None

            # For reset the status of the manipulator
            self.task_completed_once = False
            self.The_manipurator_ready_to_go = False
            self.ready_to_go = False

        elif self.status_message == "Task execution succeeded":
            self.task_execution_succeeded_time = time.time()
            self.get_logger().info("Checking the box status.")

            # if self.launch_process:
            #     self.launch_process.send_signal(signal.SIGINT)
            #     self.get_logger().info("Terminated existing launch process.")
            #     self.launching = False
            # self.get_logger().info("Task execution succeeded.")
            # self.sub_yolo_time = None
    
    def timer_callback(self):
        # Timer for checking the status of the launch file
        if self.status_message == "The sub_yolo is running" and self.sub_yolo_time is not None:
            elapsed_time1 = time.time() - self.sub_yolo_time
            if elapsed_time1 > 10.0:
                self.get_logger().info("Restarting the launch file due to timeout.")

                if self.launch_process:
                    self.launch_process.send_signal(signal.SIGINT)
                    self.launch_process.wait() #Wait for the process to terminate completely
                    self.get_logger().info("Terminated existing launch process.")
                    self.launching = False

                if hasattr(self, 'last_launch_file'):
                    self.launch_subprocess(self.last_launch_file)

                self.sub_yolo_time = None
                self.task_execution_succeeded_time = None
                self.time_to_ready_to_go = None

        # Check if Task execution succeeded but the box is not in place it will be restarted the launch file
        elif self.status_message == "Task execution succeeded" and self.box_status != "BOX" and self.task_execution_succeeded_time is not None:
            elapsed_time2 = time.time() - self.task_execution_succeeded_time
            if elapsed_time2 > 5.0:
                self.get_logger().info("Restarting the launch file due to BOX not in place.")
                if self.launch_process:
                    self.launch_process.send_signal(signal.SIGINT)
                    self.launch_process.wait() #Wait for the process to terminate completely
                    self.get_logger().info("Terminated existing launch process.")
                    self.launching = False

                if hasattr(self, 'last_launch_file'):
                    self.launch_subprocess(self.last_launch_file)
                self.sub_yolo_time = None
                self.task_execution_succeeded_time = None
        
        # Check if the task execution succeeded and the box is in place it will start mtc_ready_to_go.launch.py
        elif self.status_message == "Task execution succeeded" and self.box_status == "BOX" and not self.task_completed_once:
            if self.launch_process:
                self.launch_process.send_signal(signal.SIGINT)
                self.launch_process.wait() #Wait for the process to terminate completely
                self.launching = False
            self.launch_process = subprocess.Popen(['ros2', 'launch', 'mtc_config1', 'mtc_ready_to_go.launch.py'])
            self.get_logger().info("Task execution succeeded and BOX is placed in place.")
            self.task_execution_succeeded_time = None
            self.sub_yolo_time = None
            self.task_completed_once = True
            self.time_to_ready_to_go = time.time()

        # Check if mtc_ready_to_go.launch.py is running but the lunch file is Cached it will be restart mtc_ready_to_go.launch.py
        elif self.status_message == "Task execution succeeded" and not self.ready_to_go and self.time_to_ready_to_go is not None:  
            elapsed_time3 = time.time() - self.time_to_ready_to_go
            if elapsed_time3 > 5.0:
                if self.launch_process:
                    self.launch_process.send_signal(signal.SIGINT)
                    self.launch_process.wait() #Wait for the process to terminate completely
                    self.launching = False
                self.ready_to_go = True
                self.launch_process = subprocess.Popen(['ros2', 'launch', 'mtc_config1', 'mtc_ready_to_go.launch.py'])
                self.get_logger().info("The ready to go error and it will be restarted.")

        # Check the status of the manipulator that it is ready to go
        elif self.status_message == "The manipulator ready to go" and not self.The_manipurator_ready_to_go:
            if self.launch_process:
                self.launch_process.send_signal(signal.SIGINT)
                self.launch_process.wait() #Wait for the process to terminate completely
                self.launching = False
            self.get_logger().info("Manipulator is ready to go.")
            self.task_execution_succeeded_time = None
            self.sub_yolo_time = None
            self.time_to_ready_to_go = None
            self.The_manipurator_ready_to_go = True
            self.stop_and_go = False
            self.box_status = ""
            self.current_sign_status = "green"
        
    # This function will be called when the parking sign is detected
    def trigger_parking_sign_action(self):
        self.launch_subprocess('mtc_pick.launch.py')
    
    def trigger_place_sign_action(self):
        self.launch_subprocess('mtc_place.launch.py')
        
    
    def launch_subprocess(self, launch_file):
        if self.launching:
            self.get_logger().info("Launch already in progress, skipping.")
            return
        self.launching = True
        self.stop_and_go = True # To stop the car when the launch file is running
        self.last_launch_file = launch_file
        self.launch_process = subprocess.Popen(['ros2', 'launch', 'mtc_config1', launch_file])
        self.get_logger().info(f"Launched {launch_file} successfully.")

def main(args=None):
    rclpy.init(args=args)
    node = RealRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




   # def preProcess(self, img):
    #     # img = img[60:, :]
    #     img = img[50:, :]
    #     # img = img[5:105,:,:]                    # Crop the image
    #     # img = img[30:120,:,:]                    # Crop the image
    #     # img = img[30:120,0:150,:]                    # Crop the image
    #     img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    #     img = cv2.GaussianBlur(img,  (3, 3), 0)
    #     img = cv2.resize(img, (200, 66))
    #     img = img/255
    #     return img


    #### For LSTM self driving model
                # pre_img = self.preprocess(img)
                # self.seq_buffer.append(pre_img)
                # if len(self.seq_buffer) == self.seq_len:
                #     input_tensor = np.expand_dims(np.array(self.seq_buffer), axis=0).astype(np.float32)
                #     steering = self.self_driving_model.predict(input_tensor, verbose=0)[0][0] 
                #     # self.get_logger().info(f'Steering predicted: {steering:.2f}')
                #     alpha = 0.6
                #     smoothed_angle = alpha * steering + (1 - alpha) * self.last_smoothed_angle
                #     self.last_smoothed_angle = smoothed_angle
                #     self.angle = self.denormalize_clamped(smoothed_angle)
                #     self.get_logger().info(f"Input shape: {self.angle}")

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