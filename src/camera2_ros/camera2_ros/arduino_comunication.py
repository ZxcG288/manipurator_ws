import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import JointState
import re  # ใช้ Regular Expressions เพื่อแยกข้อมูล
import math

import serial
import struct
from interface_manipurator.msg import SpeedAngle
#import numpy as np180
import time

class Arduino_comunication(Node):
    def __init__(self):
        super().__init__('arduino_comunication')
        self.ser1 = serial.Serial('/dev/ttyUSB1', 9600, timeout=2)
        self.ser2 = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        self.car_subscriber = self.create_subscription(
            SpeedAngle,
            "/SpeedAngle",
            self.car_subscriber_callback,
            10
        )

        #Subscribe to the topic from web controller
        self.web_subscription = self.create_subscription(
            String,
            '/web_topic',
            self.web_subscription_callback,
            10)
        
        #Create publisher for collecting data from joystick for trainning
        self.collecting_data_publisher = self.create_publisher(
            Int32,
            '/collecting_data',
            10
        )

        self.joint_names = [
            'arm1_joint','arm2_joint','headbase_joint','headrotation_joint',
            'left_gear_joint','rotating_base_joint','right_gear_joint'
        ]

        self.angle = 0
        self.data1 = struct.pack('<6h', 180, 60, 180, 0, 90, 90)
        # self.data1 = struct.pack('<6h', 180, 55, 180, 0, 90, 90)
        self.data2 = struct.pack('<3h', 0, 0, 90)
        self.y = 0
        
        
        #publish the data to the topic "/collecting_data"
        self.information_to_arduino_timer = self.create_timer(0.1, self.information_to_arduino_callback)
        self.data_publish = self.create_timer(0.01, self.data_publish_callback)
  
    def car_subscriber_callback(self, msg):
        speed = msg.speed
        angle = msg.angle
        speedLeft = 0
        speedRight = 0

        if angle > 70 and angle < 110:
            speedLeft = speed
            speedRight = speed 
        elif angle < 70:
            speedLeft = int(speed * 0.6)
            speedRight = int(speed)
        elif angle > 110:
            speedLeft = int(speed)
            speedRight = int(speed * 0.6)

        angle = int(msg.angle)
        self.data2 = struct.pack('<3h', speedLeft,speedRight, angle)
        # self.get_logger().info(f'Speed: {speed}, Angle: {angle}')
        # self.get_logger().info(f'{self.data2}')
        # self.get_logger().info(f"Receive: {speed}, {angle}")
        # time.sleep(0.1)

    
    def joint_states_callback(self, msg):
        def map_value(value, from_low, from_high, to_low, to_high):# Map function similar to Arduino's map function
            return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
        A1 = int(map_value(msg.position[6], -3.14, 0, 0, 180)) #this is the rotating base joint
        A2 = int(map_value(msg.position[0], -1.57, 1.57, 0, 180)) #this is the arm1 joint
        A3 = int(map_value(msg.position[1], -1.57, 1.57, 0, 180)) #this is the arm2 joint
        A4 = int(map_value(msg.position[2], -1.57, 1.57, 180, 0)) #this is the headbase joint
        A5 = int(map_value(msg.position[3], -1.57, 1.57, 0, 180)) #this is the headrotation joint
        A6 = int(map_value(msg.position[4], 0, 1.57, 90, 0)) #this is the left_gear joint and right_gear joint for the gripper
        #self.get_logger().info(f'{A1}, {A2}, {A3}, {A4}, {A5}, {A6}')
        self.data1 = struct.pack('<6h', A1, A2, A3, A4, A5, A6)
        
    def web_subscription_callback(self, msg):
        pass
        # #self.get_logger().info(f'receive: "{msg.data}"')
        # # ใช้ regex เพื่อแยกค่าตัวเลข
        # pattern = r'Joystick X: ([\d\.-]+), Y: ([\d\.-]+), A1: (\d+), A2: (\d+), A3: (\d+), A4: (\d+), A5: (\d+)'
        # match = re.search(pattern, msg.data)

        # if match:
        #     x = float(match.group(1))
        #     y = float(match.group(2))
        #     A1 = int(match.group(3))
        #     A2 = int(match.group(4))
        #     A3 = int(match.group(5))
        #     A4 = int(match.group(6))
        #     A5 = int(match.group(7))

        #     # แสดงผลข้อมูลที่แยกได้
        #     #self.get_logger().info(f'receive X: {x}, Y: {y}, A1: {A1}, A2: {A2}, A3: {A3}, A4: {A4}, A5: {A5}')

        #     def map_value(value, from_low, from_high, to_low, to_high): #It works like map function in Arduino
        #         return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
        #     if y < 0:
        #         # map y from [-1, 0) to [-80, -50]
        #         speed = int(map_value(y, -1, 0, -50, -50))#-50, -40, -100, -90
        #     elif y > 0:
        #         # map y from (0, 1] to [50, 80]
        #         speed = int(map_value(y, 0, 1, 50, 50))#40, 50, 90, 100
        #     else:
        #         speed = 0  # y == 0 

           
        #     angle = 0
        #     if y == 0:
        #         ang = 0  # avoid division by zero error
        #     else:
        #         ang = int(math.degrees(math.atan(x / y)))  # convert values to degrees
        #     if speed > 0:
        #         angle = int(map_value(ang, -75, 75, 60, 120))  # adjust angle for positive speed
        #     elif speed < 0:
        #         angle = int(map_value(ang, -75, 75, 120, 60))  # adjust angle for negative speed
        #     elif speed == 0:  # set angle 90 because it's the middle of the car
        #         angle = 90
        #     self.get_logger().info(f'SPEED: {speed},ANGLE: {angle}')
        #     data = struct.pack('<2h', speed, angle)
        #     self.data2 = data
        #     self.angle = angle
        #     self.y = y
            
            
        # else:
        #     self.get_logger().warn("Wrong")

    def information_to_arduino_callback(self):
        self.ser1.write(self.data1)
        self.ser1.flush()
        self.ser2.write(self.data2)
        self.ser2.flush()
        
    def data_publish_callback(self):
        pass
        # msg = Int32()
        # msg.data = self.angle
        # if self.y > 0:
        #     self.collecting_data_publisher.publish(msg)
        #     self.get_logger().info(f'Publishing: "{self.angle}"')
            
        
def main(args=None):
    rclpy.init(args=args)
    node = Arduino_comunication()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#self.get_logger().info(f'{data1}')
        # # self.ser2.write(data2)
        # # self.ser2.flush()

        # position = np.array(msg.position) * (180 / 3.14159)
        # position = np.clip(position, 0, 180).astype(int)  # จำกัดค่าและแปลงเป็น int
        # formatted_position = ','.join(map(str, position))  #string
        # serial_data = f'[{formatted_position}]\n'
        # self.get_logger().info(f'Sending: {serial_data}')
        # self.ser.write(serial_data.encode())  #sent the value to arduino
        # self.ser.flush()
