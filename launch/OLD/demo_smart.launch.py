import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument

# Launch stage control in mcp mode
# Remember to launch PlusServer connected to Aurora in another terminal
# Remember to run keypress node (trajcontrol package) in another terminal

def generate_launch_description():

    # Use smart template node
    robot = Node(
        package="smart_template",
        executable="template",
    )  

    # Use mri tracking interface
    interface = Node(
        package = "smart_template",
        executable = "mri_tracking_interface",
    )

    # # Create a server bridge
    # igtl_bridge = Node(
    #     package="ros2_igtl_bridge",
    #     executable="igtl_node",
    #     parameters=[
    #         {"RIB_server_ip":"localhost"},
    #         {"RIB_port": 18944},
    #         {"RIB_type": "server"}
    #     ]
    # )

    # Use manual controller
    control = Node(
        package = "trajcontrol",
        executable = "controller_manual_smart",
        parameters = [
            {"motion_step": 1.0}
            ]
    )

    # # Save data to filename defined by user
    # save_file = Node(
    #     package = "trajcontrol",
    #     executable = "save_file",
    #     parameters =[{"filename": LaunchConfiguration('filename')}]
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            "filename",
            default_value = "my_data",
            description = "File name to save .csv file with experimental data"
        ),
        actions.LogInfo(msg = ["filename: ", LaunchConfiguration('filename')]),
        robot,
        interface,
        # igtl_bridge,
        control
    ])
