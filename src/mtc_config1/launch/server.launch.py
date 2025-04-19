from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rosbridge_launch_file = os.path.join(
        get_package_share_directory('rosbridge_server'), 
        'launch', 
        'rosbridge_websocket_launch.xml'
    )
    start_web = ExecuteProcess(
        cmd=["python3", "-m", "http.server", "8080"],
        cwd=os.path.expanduser("~/Temp/"),
        shell=True,
        output="screen",
    )
    # node_listener = Node(
    #     package="robot_manipirator",
    #     executable="web_test1_node",
    #     output="screen",
    # )
    

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch_file)
        ),
        start_web,
        # node_listener,

    ])
