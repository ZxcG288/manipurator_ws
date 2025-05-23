import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import re  # ใช้ Regular Expressions เพื่อแยกข้อมูล
import math
import serial
import struct
import time

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber') 
        #self.ser2 = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)

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
        self.angle = 0

        self.web_subscription_callback  #prevent unused variable warning

        self.timer = self.create_timer(0.033, self.timer_callback) # using it equal to the camera publsher at 30 fps

    def web_subscription_callback(self, msg):
        #self.get_logger().info(f'receive: "{msg.data}"')
        #ใช้ regex เพื่อแยกค่าตัวเลข
        pattern = r'Joystick X: ([\d\.-]+), Y: ([\d\.-]+), A1: (\d+), A2: (\d+), A3: (\d+), A4: (\d+), A5: (\d+)'
        match = re.search(pattern, msg.data)

        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            A1 = int(match.group(3))
            A2 = int(match.group(4))
            A3 = int(match.group(5))
            A4 = int(match.group(6))
            A5 = int(match.group(7))

            # แสดงผลข้อมูลที่แยกได้
            # self.get_logger().info(f'receive X: {x}, Y: {y}, A1: {A1}, A2: {A2}, A3: {A3}, A4: {A4}, A5: {A5}')

            def map_value(value, from_low, from_high, to_low, to_high): #It works like map function in Arduino
                return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
            speed = int(map_value(y, -1, 1, -80, 80)) #300 is the max speed of the car but use 80 for AI trainning model 
            angle = 0
            if y == 0:
                ang = 0  # avoid division by zero error
            else:
                ang = int(math.degrees(math.atan(x / y)))  # convert values to degrees
            if speed > 0:
                angle = int(map_value(ang, -75, 75, 45, 135))  # adjust angle for positive speed
            elif speed < 0:
                angle = int(map_value(ang, -75, 75, 135, 45))  # adjust angle for negative speed
            elif speed == 0:  # set angle 90 because it's the middle of the car
                angle = 90
            self.angle = angle
            self.get_logger().info(f'SPEED: {speed},ANGLE: {angle}')
            data = struct.pack('<2h', speed, angle)
            #self.get_logger().info(f'{data}')
            # self.ser2.write(data)
            # self.ser2.flush()
            # time.sleep(0.1)

        else:
            self.get_logger().warn("Wrong")

    def timer_callback(self):
        msg = Int32()
        msg.data = self.angle
        self.collecting_data_publisher.publish(msg)
        # self.get_logger().info(f'Publishing: "{self.angle}"')

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
