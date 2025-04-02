#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class ServoControl(Node):

    def __init__(self):
        super().__init__('servo_control')
        self.publisher_ = self.create_publisher(Int16, 'servo_angle', 10)
        #  ไม่ต้องใช้ timer ในกรณีนี้
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def get_angle_from_keyboard(self):
        while True:
            try:
                angle = int(input("ป้อนมุม Servo (0-180): "))
                if 0 <= angle <= 180:
                    return angle
                else:
                    print("ค่ามุมต้องอยู่ระหว่าง 0-180 องศา")
            except ValueError:
                print("กรุณาป้อนตัวเลข")

    def publish_angle(self, angle):
        msg = Int16()
        msg.data = angle
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()

    while True:
        angle = servo_control.get_angle_from_keyboard()
        servo_control.publish_angle(angle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servo_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()