#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
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

  // this use for status_pub of manipurator
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  void publishStatus(const std::string& msg) {
    std_msgs::msg::String status_msg;
    status_msg.data = msg;
    status_pub_->publish(status_msg);
  }
  
  // Callback function for position update
  void positionCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr position_sub_;  // Subscriber to position
  geometry_msgs::msg::Pose current_position_;  // To store current position

  bool position_received_;  // Flag to indicate if position is received
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node_test", options) },
    position_received_(false) // Set the flag to false initially
{
  // Subscribe to the position topic
  position_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
    "/position_topic", 10, std::bind(&MTCTaskNode::positionCallback, this, std::placeholders::_1) 
  );
  // Publisher for status messages
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/manipurator_information", 10
  );
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::positionCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  current_position_ = *msg;  // Update the current position with the received message
  position_received_ = true;  // Set the flag to true

  // Log the position values
  // RCLCPP_INFO(LOGGER, "Received position: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
  //             current_position_.position.x,
  //             current_position_.position.y,
  //             current_position_.position.z,
  //             current_position_.orientation.z);
}

void MTCTaskNode::setupPlanningScene()
{
  // RCLCPP_INFO(LOGGER, "Setting up planning scene with object at x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
  //             current_position_.position.x,
  //             current_position_.position.y,
  //             current_position_.position.z,
  //             current_position_.orientation.z);

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";

  // Define shape of the object
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.04, 0.04, 0.04 };  // Set dimensions of the object

  // Use the current position received from the subscriber
  geometry_msgs::msg::Pose pose;
    
  // กำหนดตำแหน่ง (Position)
  pose.position.x = current_position_.position.x;
  pose.position.y = current_position_.position.y;
  pose.position.z = current_position_.position.z;

  // Define quaternion from Roll, Pitch, Yaw (radians)
  // tf2::Quaternion q;
  // q.setRPY(0, 0, 0);  // Adjust for tilt if needed
  pose.orientation = current_position_.orientation;  // Translate to geometry_msgs::msg::Quaternion

  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

//set up work

// void MTCTaskNode::doTask()
// {
//   while (!position_received_) {
//     RCLCPP_INFO(LOGGER, "Waiting for position data...");
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   }
    
//     setupPlanningScene();
//     task_ = createTask();

//     try
//     {
//       task_.init();
//     }
//     catch (mtc::InitStageException& e)
//     {
//       RCLCPP_ERROR_STREAM(LOGGER, e);
//       return;
//     }
  
//     if (!task_.plan(1)) //Set the number of capture simulations
//     {
//       RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
//       return;
//     }
//     task_.introspection().publishSolution(*task_.solutions().front());
  
//     auto result = task_.execute(*task_.solutions().front());
//     if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
//     {
//       RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
//       return;
//     }
  
//     return;
// }
void MTCTaskNode::doTask() {
  while (true) {
    while (!position_received_) {
      RCLCPP_INFO(LOGGER, "Waiting for position data...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    publishStatus("The task has been stated");
    setupPlanningScene();
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
      std::this_thread::sleep_for(std::chrono::seconds(7)); //for wait in seconds
      continue;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      publishStatus("Task execution failed");
      std::this_thread::sleep_for(std::chrono::seconds(7)); //for wait in seconds
      continue;
    }

    //if success it will break the loop
    RCLCPP_INFO(LOGGER, "Task execution succeeded");
    publishStatus("Task execution succeeded");
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
  auto stage_open_hand = //open the gripper 
      std::make_unique<mtc::stages::MoveTo>("open for pick", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);//setting what type do you want to move such arm_group_name, hand_group_name, hand_frame
  stage_open_hand->setGoal("open");// this stage from .srdf file
  task.add(std::move(stage_open_hand)); 
  
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>( //stage connector between open the gripper stage and pick object stage
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0); //set time out if arm can't do it within time
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
{
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object"); //
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
  {
    // Sample grasp pose and grip angle
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open"); //open the grapper
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
      
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q =  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = -0.13; //distance between end-effector and object
    
    //grasp_frame_transform.translation().x() = 0.125;
    //grasp_frame_transform.translation().y() = 0.135;
      // Compute IK
      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));
    }
    // Prepare to grasp the object
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(), //setting for hand can close collape with object
                            true);
      grasp->insert(std::move(stage));
    }
    {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close_pick"); //close the grapper
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
}
                                  
  //for place item
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre place1", interpolation_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setGoal("pre_place1");
      task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre place2", interpolation_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setGoal("pre_place2");
      task.add(std::move(stage));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
  {
    // Sample place pose
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "place_pose");
    stage->setObject("object");

    //Set position to place 
    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "world";
    target_pose_msg.pose.position.x = -0.055; //set position for place the object
    target_pose_msg.pose.position.y = -0.1;
    target_pose_msg.pose.position.z = 0.14;
    //target_pose_msg.pose.orientation.w = 1.0;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(0.1);
    wrapper->setIKFrame("object");
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place->insert(std::move(wrapper));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    place->insert(std::move(stage));
  }

  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage->allowCollisions("object",
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
    place->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject("object", hand_frame);
    place->insert(std::move(stage));
  }

  task.add(std::move(place));
}

 //Ready to go 
  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("ready to go", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("try_place");
    task.add(std::move(stage));
  }
  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close"); //close the grapper
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

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0; //if all of code is true 
}