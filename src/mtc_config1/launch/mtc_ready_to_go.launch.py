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

    ready_to_go = Node(
        package="mtc_config1",
        executable="mtc_ready_to_go",
        output="screen",
        parameters=[moveit_params],  # Pass the dictionary of parameters here
    )
    
    return LaunchDescription(
        [
            ready_to_go,
        ]
    )
