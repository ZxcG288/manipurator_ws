import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Build the MoveIt configurations
    moveit_config = MoveItConfigsBuilder("urdf1", package_name="config1").to_moveit_configs()

    # Convert MoveItConfigs to parameters dictionary
    moveit_params = moveit_config.to_dict()
    
    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_config1",
        executable="place_node",
        output="screen",
        parameters=[moveit_params],  # Pass the dictionary of parameters here
    )

    return LaunchDescription([pick_place_demo])
