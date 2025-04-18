import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from interface_manipurator.msg import SpeedAngle
from geometry_msgs.msg import Pose 

class PickPlaceController(Node):
    def __init__(self):
        super().__init__("pick_place_controller")
        self.status_message = ""
        self.stop_time = None  # Initialize stop_time attribute to None

        self.status_sub = self.create_subscription(
            String,
            "/manipurator_information",
            self.status_callback,
            10
        )
        self.car_publisher = self.create_publisher(
            SpeedAngle, 
            "/SpeedAngle", 
            10
        )
        self.sub_position_topic = self.create_subscription(
            Pose,
            "/position_topic",
            self.sub_position_topic_callback,
            10,
        )
        self.car_timer = self.create_timer(0.01, self.car_publish)

    def sub_position_topic_callback(self, msg):
        self.current_position = msg.position.y
        
    def status_callback(self, msg):
        self.status_message = msg.data

    def car_publish(self):
        msg = SpeedAngle()
        speed = 0  # Default speed
        angle = 0   # Default angle

        if hasattr(self, 'status_message'):
            if hasattr(self, 'current_position'):
                if self.status_message == "Task planning failed":
                    if self.stop_time is None:
                        self.stop_time = self.get_clock().now().seconds_nanoseconds()[0]

                    stop_duration = self.get_clock().now().seconds_nanoseconds()[0] - self.stop_time
                    if stop_duration < 2: #loop for 2 sec
                        if self.current_position > 0.08: #because 0.08 is on the middle of the RC car and 0.00 is almost the front of RC car
                            speed = -50  # -10 because the object on the backside of RCcar so it has to Reverse the car backwards
                        elif self.current_position < 0.08:
                            speed = 50
                    else:
                        self.status_message = ""  # Reset the status message
                        self.stop_time = None  # Reset stop_time to allow future changes

                elif self.status_message == "Task execution succeeded":
                    rclpy.shutdown()

        msg.speed = speed
        msg.angle = angle
        self.car_publisher.publish(msg)
        self.get_logger().info(f"publish:{speed}, {angle}")

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
