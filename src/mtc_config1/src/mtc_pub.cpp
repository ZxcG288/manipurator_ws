#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PositionPublisherNode : public rclcpp::Node
{
public:
  PositionPublisherNode()
  : Node("position_publisher_node")
  {
    // สร้าง publisher ที่ส่งข้อความประเภท Pose ไปที่ topic "/position_topic"
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/position_topic", 10);

    // เรียกฟังก์ชัน publish_position() ทุก 1 วินาที
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PositionPublisherNode::publish_position, this)
    );
  }

private:
  void publish_position()
  {
    geometry_msgs::msg::Pose pose;
    
    // กำหนดตำแหน่ง (Position)
    pose.position.x = -0.25;
    pose.position.y = 0.05;
    pose.position.z = 0.07;
    pose.orientation.z = 0.3;
    // กำหนด orientation (ทิศทาง)
    // tf2::Quaternion q;
    // q.setRPY(0, 0, 0.3); //tf2::toMsg(q); // Roll, Pitch, Yaw
    
    // ส่งตำแหน่งไปยัง topic "/position_topic"
    publisher_->publish(pose);
    RCLCPP_INFO(this->get_logger(), "Published position: [%f, %f, %f, %f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.z);
  }
  
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;  // Publisher สำหรับ Pose
  rclcpp::TimerBase::SharedPtr timer_;  // Timer ที่ใช้เรียกฟังก์ชัน publish_position
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
