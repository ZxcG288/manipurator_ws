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
        self.ser1 = serial.Serial('/dev/ttyUSB1', 9600, timeout=2)
        self.ser2 = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)

        self.subscription = self.create_subscription(
            String,
            '/web_topic',
            self.listener_callback,
            10)
        
        #Create publisher for collecting data from joystick for trainning
        self.collecting_data_publisher = self.create_publisher(
            Int32,
            '/collecting_data',
            10
        )

        self.subscription  # ป้องกัน unused variable warning
        self.data1 = struct.pack('<6h', 180, 60, 180, 0, 90, 90)
        # self.data1 = struct.pack('<6h', 180, 67, 180, 15, 90, 90)
        # self.data1 = struct.pack('<6h', 180, 72, 180, 15, 90, 90)
        self.data2 = struct.pack('<3h', 0, 0, 90)
        self.angle = 90

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.data_publish = self.create_timer(0.1, self.data_publish_callback)
        

    def listener_callback(self, msg):
        # Regular expression pattern
        pattern = r'Joystick X: ([\d\.-]+), Y: ([\d\.-]+), A1: (\d+), A2: (\d+), A3: (\d+), A4: (\d+), A5: (\d+), Speed: ([\d\.-]+)'
        
        # Search for the pattern in the message data
        match = re.search(pattern, msg.data)
        
        if match:
            # Extract values from the match object
            x = float(match.group(1))
            y = float(match.group(2))
            A1 = int(match.group(3))
            A2 = int(match.group(4))
            A3 = int(match.group(5))
            A4 = int(match.group(6))
            A5 = int(match.group(7))
            speed = int(match.group(8))

            # แสดงผลข้อมูลที่แยกได้
            #self.get_logger().info(f'receive X: {x}, Y: {y}, A1: {A1}, A2: {A2}, A3: {A3}, A4: {A4}, A5: {A5}')

            def map_value(value, from_low, from_high, to_low, to_high): #It works like map function in Arduino
                return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
            
            angle = 90
            speedLeft = 0
            speedRight = 0
            
            ang = int(math.degrees(math.atan2(x, y)))  # ใช้ atan2 แทน atan

            # Clamp ค่า ang ให้อยู่ในช่วง [-75, 75] เพื่อความปลอดภัย
            ang = max(-75, min(75, ang))

            # Map มุมให้เป็น 60-90 สำหรับซ้าย และ 90-120 สำหรับขวา
            if ang < 0:
                angle = int(map_value(ang, -75, 0, 45, 90))
            elif ang > 0:
                angle = int(map_value(ang, 0, 75, 90, 135))
            else:
                angle = 90
                
            if angle > 70 and angle < 110:
                speedLeft = speed
                speedRight = speed 
            elif angle < 70:
                speedLeft = int(speed * 0.6)
                speedRight = int(speed)
            elif angle > 110:
                speedLeft = int(speed)
                speedRight = int(speed * 0.6)

            self.angle = angle

            
            #self.get_logger().info(f'SPEED: {speed},ANGLE: {angle}')
           # self.get_logger().info(f'Sending to Arduino: speedLeft={speedLeft}, speedRight={speedRight}, angle={angle}')

            #self.data1 = struct.pack('<6h', A1, A2, A3, A4, A5, 90)
            self.data2 = struct.pack('<3h', speedLeft, speedRight , angle)
            
        else:
            self.get_logger().warn("Wrong")

    def timer_callback(self):
        #self.get_logger().info(f'{self.data2}')
        self.ser1.write(self.data1)
        self.ser1.flush()
        self.ser2.write(self.data2)
        self.ser2.flush()

    def data_publish_callback(self):
        msg = Int32()
        msg.data = self.angle
        self.collecting_data_publisher.publish(msg)
        #self.get_logger().info(f'Publishing: "{self.angle}"')
            
        
def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
