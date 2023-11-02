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
# Remember to launch bridge server in another terminal
# Remember to run keypress node (trajcontrol package) in another terminal

def generate_launch_description():

    # Use smart template node
    robot = Node(
        package="smart_template",
        executable="template",
    )  

    # Use mri tracking interface
    mritracking_interface = Node(
        package = "smart_template",
        executable = "mri_tracking_interface",
    )

    # Use keyboard interface
    initialization = Node(
        package = "smart_template",
        executable = "initialization",
    )

    # Use planning interface
    planning= Node(
        package = "trajcontrol",
        executable = "planning",
        parameters = [
            {"use_slicer": False},
            {"air_gap": 1.0},
            {"insertion_length": 15.0}
            ]
    )

    # Use manual controller
    manual_control = Node(
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
        # DeclareLaunchArgument(
        #     "filename",
        #     default_value = "my_data",
        #     description = "File name to save .csv file with experimental data"
        # ),
        # actions.LogInfo(msg = ["filename: ", LaunchConfiguration('filename')]),
        robot,
        initialization,
        mritracking_interface,
        planning,
        manual_control,
        # save_file
    ])
