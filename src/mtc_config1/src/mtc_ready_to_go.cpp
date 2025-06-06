#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2/LinearMath/Quaternion.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

//std

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_config1");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  rclcpp::Node::SharedPtr node_;
  mtc::Task createTask();
  mtc::Task task_;

  // this use for status_pub of manipurator
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  void publishStatus(const std::string& msg) {
    std_msgs::msg::String status_msg;
    status_msg.data = msg;
    status_pub_->publish(status_msg);
  }
  
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_ready_to_go", options) }
{
  // Publisher for status messages
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/manipurator_information", 10
  );
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

//set up work
void MTCTaskNode::doTask() 
{
  while (true) {
    
    task_ = createTask();

    try {
      task_.init();
    } catch (mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      publishStatus("Task initialization failed: " + std::string(e.what()));
      continue;
    }

    if (!task_.plan(1)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      publishStatus("Task planning failed"); //if planning failed it will move the car for change the position
      std::this_thread::sleep_for(std::chrono::seconds(1)); 
      continue;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      publishStatus("Task execution failed");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    //if success it will break the loop
    RCLCPP_INFO(LOGGER, "The manipulator ready to go");
    publishStatus("The manipulator ready to go");
    break;
  }
}
//coding for pick and place object

mtc::Task MTCTaskNode::createTask() //this parameter for create Task to pick and place the object
{
    mtc::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "arm";
    const auto& hand_group_name = "hand";
    const auto& hand_frame = "headrotation_link";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // Disable warnings for this line, as it's a variable that's set but not used in this example
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    #pragma GCC diagnostic pop

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_); //use OMPL for solvers
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>(); //for simple task
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>(); // CartesianPath is used to move the end effector in a straight line in Cartesian space.

    //for movement speed of manipurator
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner->setStepSize(.01);

  //for pick item
    {
    auto stage = std::make_unique<mtc::stages::MoveTo>("start", interpolation_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setGoal("ready");
      task.add(std::move(stage));
    }
                        
  return task;
}

int main(int argc, char** argv) //main function
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });


    
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0; //if all of code is true 
}