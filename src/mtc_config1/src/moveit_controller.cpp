#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>

class TrajectoryMonitor : public rclcpp::Node
{
public:
    TrajectoryMonitor() : Node("trajectory_monitor")
    {
        // Subscriber ที่ฟังสถานะจาก Trajectory Execution
        status_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "trajectory_status", 10,
            std::bind(&TrajectoryMonitor::statusCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber_;
    std::mutex execution_state_mutex_;

    void statusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // ใช้ mutex เพื่อป้องกันการเข้าถึงข้อมูลร่วมกัน
        std::scoped_lock slock(execution_state_mutex_);

        if (msg->data == "SUCCEEDED")
        {
            // รอให้หุ่นยนต์หยุด (สมมติว่ามีฟังก์ชัน waitForRobotToStop)
            RCLCPP_INFO(this->get_logger(), "Robot stopped, calculation starts...");

            // ตัวอย่าง: พิมพ์ 1+1=2
            int result = 1 + 1;
            RCLCPP_INFO(this->get_logger(), "1 + 1 = %d", result);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received unexpected status: %s", msg->data.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryMonitor>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(node->get_logger(), "Unknown exception caught");
    }

    rclcpp::shutdown();
    return 0;
}
