import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Launch system in manual control mode
# Remember to run keypress node (trajcontrol package) in another terminal
# If use_slicer: launch bridge server in another terminal 

def generate_launch_description():

    ld = LaunchDescription()

    arg_use_slicer = DeclareLaunchArgument(
        "use_slicer",
        default_value = "True",
        description = "Use 3DSlicer (True) or stand-alone mode (False)"
    )

    arg_needle_length = DeclareLaunchArgument(
        "needle_length",
        default_value = "20.0",
        description = "Set the needle total lenght [mm]"
    )

    arg_insertion_length = DeclareLaunchArgument(
        "insertion_length",
        default_value = "10.0",
        description = "Set total insertion lenght inside tissue [mm]. Valid only if use_slicer is True"
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
            {"use_slicer": LaunchConfiguration('use_slicer')},
            {"needle_length": LaunchConfiguration('needle_length')}
        ]
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
    ld.add_action(arg_use_slicer)
    ld.add_action(arg_needle_length)
    ld.add_action(arg_insertion_length)
    ld.add_action(arg_insertion_step)
    
    ld.add_action(smart_needle_interface)
    ld.add_action(planning)
    ld.add_action(manual_control)
    
    return ld
