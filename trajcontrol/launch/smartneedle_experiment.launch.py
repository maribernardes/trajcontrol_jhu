import os

from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node

# Launch trajcontrol environment for experiment (with 3D Slicer)

def generate_launch_description():

    ld = LaunchDescription()

    arg_insertion_length = DeclareLaunchArgument(
        "insertion_length",
        default_value = "10.0",
        description = "Set total insertion lenght inside tissue [mm]. Used only if use_slicer is True"
    )

    arg_insertion_step = DeclareLaunchArgument(
            "insertion_step",
            default_value = "10.0",
            description = "Insertion step size in mm"
    )

    # Use SmartNeedle interface
    smart_needle_interface = Node(
        package = "trajcontrol",
        executable = "smart_needle_interface",
        parameters = [
            {"use_slicer": True},
        ]
    )

    # Use planning interface
    # air_gap is not used because use_slicer is True
    planning= Node(
        package = "trajcontrol",
        executable = "planning",
        parameters = [
            {"use_slicer": True},
            {"insertion_length": LaunchConfiguration('insertion_length')}
        ]
    )

    # Use manual controller
    # to be replaced by controller_mpc
    manual_control = Node(
        package = "trajcontrol",
        executable = "controller_manual_smart",
        parameters = [
            {"lateral_step": 1.0},
            {"insertion_step": LaunchConfiguration('insertion_step')},
            {"wait_init": True}
        ]
    )

    # # Save data to filename defined by user
    # save_file = Node(
    #     package = "trajcontrol",
    #     executable = "save_file",
    #     parameters =[{"filename": LaunchConfiguration('filename')}]
    # )

    # Include launch arguments
    ld.add_action(arg_insertion_length)
    ld.add_action(arg_insertion_step)
    
    ld.add_action(smart_needle_interface)
    ld.add_action(planning)
    ld.add_action(manual_control)
    
    return ld
