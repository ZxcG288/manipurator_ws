#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_config1");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("start_mtc", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

//set up work
void MTCTaskNode::doTask()
{
  while (true) //using loop if planning failed 
  {
    task_ = createTask();
    
    try
    {
      task_.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      continue; //restart loop 
    }

    if (!task_.plan(1)) // Set the number of capture simulations
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      continue; //restart loop 
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      continue; //restart loop 
    }

    break; //break the loop if it's true
  }
}
// void MTCTaskNode::doTask() //Set task to do pick and place 
// {
//     task_ = createTask();

//     try
//     {
//         task_.init();
//     }
//     catch (mtc::InitStageException& e)
//     {
//         RCLCPP_ERROR_STREAM(LOGGER, e);
//         return;
//     }

//     if (!task_.plan(1)) //Set the number of capture simulations
//     {
//         RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
//         return;
//     }
//     task_.introspection().publishSolution(*task_.solutions().front());

//     auto result = task_.execute(*task_.solutions().front());
//     if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
//     {
//         RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
//         return;
//     }

//     return;
// }

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
    auto stage = std::make_unique<mtc::stages::MoveTo>("ready to go", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("try_pick");
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