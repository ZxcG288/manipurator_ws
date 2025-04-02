import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import serial


class Arduino_comunication(Node):
    def __init__(self):
        super().__init__('arduino_comunication')
        self.publisher_ = self.create_publisher(Int32, 'servo_angle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        #self.ser = serial.Serial('/dev/ttyUSB0', 9600)  # Specify correct port
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.joint_names = [
            'arm1_joint', 'arm2_joint', 'headbase_joint', 'headrotation_joint',
            'left_gear_joint', 'rotating_base_joint', 'right_gear_joint'
        ]

    def joint_states_callback(self, msg):
        joint_positions = {}
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                joint_positions[name] = position
        for joint, position in joint_positions.items():
            self.get_logger().info(f'{joint}: {position}')

    def timer_callback(self):
        # Your existing timer callback code
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Arduino_comunication()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
