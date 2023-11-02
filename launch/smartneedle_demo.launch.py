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
    smart_needle_interface = Node(
        package = "smart_template",
        executable = "smart_needle_interface",
        parameters = [
            {"use_slicer": LaunchConfiguration('use_slicer')}
        ]
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
            {"use_slicer": LaunchConfiguration('use_slicer')},
            {"air_gap": 1.0},
            {"insertion_length": LaunchConfiguration('insertion_length')}
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
        DeclareLaunchArgument(
            "use_slicer",
            default_value = "False",
            description = "Use 3DSlicer (True) or stand-alone mode (False)"
        ),
        actions.LogInfo(msg = ["use_slicer: ", LaunchConfiguration('use_slicer')]),     
        DeclareLaunchArgument(
            "insertion_length",
            default_value = "10.0",
            description = "Set total insertion lenght inside tissue [mm]. Valid only if use_slicer is True"
        ),
        actions.LogInfo(msg = ["insertion_length: ", LaunchConfiguration('insertion_length')]),     
        robot,
        initialization,
        smart_needle_interface,
        planning,
        manual_control,
        # save_file
    ])
