import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

class SteeringPlotNode(Node):
    def __init__(self):
        super().__init__('steering_plot_node')
        
        # โหลดข้อมูลไฟล์ (Base graph)
        # df = pd.read_csv("log_16.csv", header=None)
        df = pd.read_csv(
            os.path.join(
                os.environ['HOME'], 
                'neural_network_self_driving_ws', 
                'DataCollected', 
                'log_16.csv'
            ), 
            header=None
        )
        fps = 15
        self.base_time = df.index / fps
        self.base_angles = df.iloc[:, 1].astype(float)
        
        # ข้อมูล real-time
        self.rt_time = deque(maxlen=300)
        self.rt_angles = deque(maxlen=300)
        
        self.start_time = self.get_clock().now()
        
        self.subscription = self.create_subscription(
            Float32,
            '/steering_angle',
            self.listener_callback,
            10)
        
        # ตั้งค่า plot
        self.fig, self.ax = plt.subplots()
        # พล็อต base graph สีน้ำเงิน
        self.ax.plot(self.base_time, self.base_angles, color='blue', label='Base Data')
        
        # สร้าง line object สำหรับ real-time graph สีแดง (อัปเดตต่อไป)
        self.rt_line, = self.ax.plot([], [], color='red', label='Real-time Data')
        
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("Angle (Degrees)")
        self.ax.set_title("Steering Angle Over Time")
        self.ax.grid(True)
        self.ax.legend()
        
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received steering angle: {msg.data}')
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.rt_time.append(current_time)
        self.rt_angles.append(msg.data)
    
    def update_plot(self, frame):
        if len(self.rt_time) == 0:
            return self.rt_line,
        self.ax.set_xlim(0, max(self.base_time.iloc[-1], self.rt_time[-1]) + 1)
        all_time = list(self.rt_time)
        all_angles = list(self.rt_angles)
        self.rt_line.set_data(all_time, all_angles)
        return self.rt_line,

def main(args=None):
    rclpy.init(args=args)
    node = SteeringPlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
