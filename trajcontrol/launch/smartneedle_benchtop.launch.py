import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node

# Launch trajcontrol environment for benchtop tests (no 3D Slicer)

def generate_launch_description():

    ld = LaunchDescription()

    arg_air_gap = DeclareLaunchArgument(
        "air_gap",
        default_value = "1.0",
        description = "Gap between skin and needle guide"
    )

    arg_needle_length = DeclareLaunchArgument(
        "needle_length",
        default_value = "20.0",
        description = "Set the needle total lenght [mm]"
    )

    arg_insertion_length = DeclareLaunchArgument(
        "insertion_length",
        default_value = "10.0",
        description = "Set total insertion lenght inside tissue [mm]"
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
            {"use_slicer": False},
        ]
    )

    # Use planning interface
    # air_gap is used because use_slicer is False
    planning= Node(
        package = "trajcontrol",
        executable = "planning",
        parameters = [
            {"use_slicer": False},
            {"air_gap": LaunchConfiguration('air_gap')},
            {"insertion_length": LaunchConfiguration('insertion_length')}
        ]
    )

    # Use manual controller
    manual_control = Node(
        package = "trajcontrol",
        executable = "controller_manual_smart",
        emulate_tty = True,
        parameters = [
            {"lateral_step": 1.0},
            {"insertion_step": LaunchConfiguration('insertion_step')},
            {"wait_init": True}
        ]
    )

    # Include launch arguments
    ld.add_action(arg_air_gap)
    ld.add_action(arg_needle_length)
    ld.add_action(arg_insertion_length)
    ld.add_action(arg_insertion_step)
    
    ld.add_action(smart_needle_interface)
    ld.add_action(planning)
    ld.add_action(manual_control)
    
    return ld
