import os

from launch import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# Launch controller in manual mode
# No waiting for experiment initialization

def generate_launch_description():

    ld = LaunchDescription()
 
    arg_lateral_step = DeclareLaunchArgument(
            "lateral_step",
            default_value = "1.0",
            description = "Lateral step size in mm"
    )
    arg_insertion_step = DeclareLaunchArgument(
            "insertion_step",
            default_value = "10.0",
            description = "Insertion step size in mm"
    )

    # Use manual controller
    manual_control = Node(
        package = "trajcontrol",
        executable = "controller_manual_smart",
        parameters = [
            {"lateral_step": LaunchConfiguration('lateral_step')},
            {"insertion_step": LaunchConfiguration('insertion_step')},
            {"wait_init": False},
        ]
    )

    # Include launch arguments
    ld.add_action(arg_lateral_step)
    ld.add_action(arg_insertion_step)

    ld.add_action(manual_control)