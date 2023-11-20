import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo

import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import ThisLaunchFileDir

# Launch SmartTemplate in manual mode
# Remember to launch keypress node (smart_template package) in another terminal

def generate_launch_description():

    # robot = Node(
    #     package="smart_template",
    #     executable="template",
    # )  

    robot = Node(
        package="smart_template",
        executable="virtual_template",
    )  

    manual_control = Node(
        package="trajcontrol",
        executable = "controller_manual_smart",
        parameters = [{"motion_step": LaunchConfiguration('motion_step')}]
    )   

    initialization = Node(
        package="smart_template",
        executable = "initialization"
    )   

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_step",
            default_value = "1.0",
            description = "Step size in mm"
        ),
        actions.LogInfo(msg = ["motion_step: ", LaunchConfiguration('motion_step')]),
        robot, 
        initialization,
        manual_control
    ])
