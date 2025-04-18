import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction, Shutdown
import time 


#time.sleep(3)
def generate_launch_description():
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("urdf1", package_name="config1")
        .robot_description(file_path="config/urdf1.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    moveit_config = MoveItConfigsBuilder("urdf1", package_name="config1").to_moveit_configs()

    # Convert MoveItConfigs to parameters dictionary
    moveit_params = moveit_config.to_dict()

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("config1") + "/launch/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("config1"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "arm_controller",
        "hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # pick_place_demo = Node(
    #     package="mtc_config1",
    #     executable="mtc_node_test",
    #     output="screen",
    #     parameters=[moveit_params],  # Pass the dictionary of parameters here
    # )
    
    # yolo_pub = Node(
    #     package="mtc_config1",
    #     executable="mtc_pub",
    #     output="screen",
    # )
    # Timer to shutdown yolo_pub after 15 seconds (5 seconds for delay + 10 seconds for running time)
    # shutdown_yolo_pub = TimerAction(
    #     period=15.0,  # Total 15 seconds to stop yolo_pub (5 for delay + 10 for running)
    #     actions=[ExecuteProcess(
    #     cmd=["pkill", "-f", "/home/golf1234_pc/manipurator_ws/install/mtc_config1/lib/mtc_config1/mtc_pub"], 
    #     shell=True,
    #     output="screen",
    #     )],
    # )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            # pick_place_demo,
            

        ]+load_controllers  
        
        # +[
        #     TimerAction(
        #     period=5.0,  # หน่วงเวลา 5 วินาทีก่อนเริ่ม node
        #     actions=[yolo_pub]
        #     ),
        # #     shutdown_yolo_pub,
           
        # ]
    )
