import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction, Shutdown
import time 

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

    # Convert MoveItConfigs to parameters dictionary
    moveit_params = moveit_config.to_dict()

    pre_pick = Node(
        package="mtc_config1",
        executable="place_node",
        output="screen",
        parameters=[moveit_params],  # Pass the dictionary of parameters here
    )

    pick = Node(
        package="mtc_config1",
        executable="mtc_place",
        output="screen",
        parameters=[moveit_params],  # Pass the dictionary of parameters here
    )
    sub_yolo = Node(
        package="camera2_ros",
        executable="sub_yolo",
        output="screen",
    )
    
    box_dectection = Node(
        package="camera2_ros",
        executable="box_dectection_place",
        output="screen",
    )
    
    #Timer to shutdown yolo_pub after 15 seconds (5 seconds for delay + 10 seconds for running time)
    # shutdown_pre_pick = TimerAction(
    # period=5.0,  # Total 15 seconds to stop yolo_pub (5 for delay + 10 for running)
    # actions=[
    #     ExecuteProcess(
    #         cmd=["pkill", "-f", "/home/golf1234_pc/manipurator_ws/install/mtc_config1/lib/mtc_config1/place_node"],
    #         shell=True,
    #         output="screen",
    #         ),
    #     ],
    # )

    return LaunchDescription(
        [
            pre_pick,
            sub_yolo,
            TimerAction(
            period=5.5,  #for delay to start node
            actions=[pick]
            ),
            TimerAction(
            period=6.0,  
            actions=[box_dectection]
            ),
            # shutdown_pre_pick,
           
        ]
    )
