from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='camera2_ros',
             executable='sub_yolo.py',
             output='screen'),
    ])