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
        executable="start_mtc",
        output="screen",
        parameters=[moveit_params],  # Pass the dictionary of parameters here
    )
    shutdown_pre_pick = TimerAction(
    period=4.0,
    actions=[
        ExecuteProcess(
            cmd=["pkill", "-f", "/home/golf1234_pc/manipurator_ws/install/mtc_config1/lib/mtc_config1/start_mtc"],
            shell=True,
            output="screen",
            ),
        ],
    )
    sub_yolo = TimerAction(
        period=2.0,
        actions=[
            Node(
            package="camera2_ros",
            executable="sub_yolo",
            output="screen",
            ) 
        ]
    )
    pick = TimerAction(
        period=5.0,
        actions=[
            Node(
            package="mtc_config1",
            executable="mtc_node_test",
            output="screen",
            parameters=[moveit_params],  # Pass the dictionary of parameters here
            ) 
        ]
    )
    car_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
            package="robot_manipurator",
            executable="pick_place_controller",
            output="screen",
            ) 
        ]
    )
    box_dectection = TimerAction(
        period=8.0,
        actions=[
            Node(
            package="camera2_ros",
            executable="box_dectection_pick",
            output="screen",
            ) 
        ]
    )
    

    #sub_yolo = Node(
    #     package="camera2_ros",
    #     executable="sub_yolo",
    #     output="screen",
    # )
    # pick = Node(
    #     package="mtc_config1",
    #     executable="mtc_node_test",
    #     output="screen",
    #     parameters=[moveit_params],  # Pass the dictionary of parameters here
    # )
    # box_dectection = Node(
    #     package="camera2_ros",
    #     executable="box_dectection_pick",
    #     output="screen",
    # )
    #Timer to shutdown
    
    # shutdown_object = TimerAction(
    # period=28.0,
    # actions=[
    #     ExecuteProcess(
    #         cmd=["pkill", "-f", "/home/golf1234_pc/manipurator_ws/install/camera2_ros/lib/camera2_ros/sub_yolo"],
    #         shell=True,
    #         output="screen",
    #         ),
    #     ],
    # )

    return LaunchDescription(
        [
            pre_pick,
            shutdown_pre_pick,
            sub_yolo,
            pick,
            box_dectection,
            # car_controller,
            # TimerAction(
            # period=5.0,  #for delay to start node
            # actions=[pick]
            # ),
            # TimerAction(
            # period=7.0,  
            # actions=[box_dectection]
            # ),
            # shutdown_pre_pick,
            # shutdown_object,
        ]
    )
