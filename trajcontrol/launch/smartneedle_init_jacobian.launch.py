import os

from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node

# Launch trajcontrol environment for experiment (with 3D Slicer)

def generate_launch_description():

    ld = LaunchDescription()

    arg_insertion_step = DeclareLaunchArgument(
            "insertion_step",
            default_value = "5.0",
            description = "Insertion step size in mm"
    )

    arg_air_gap = DeclareLaunchArgument(
            "air_gap",
            default_value = "3.6",
            description = "Air gap size in mm"
    )

    # Use planning interface
    # air_gap is not used because use_slicer is True
    planning= Node(
        package = "trajcontrol",
        executable = "planning",
        parameters = [
            {"use_slicer": False},
            {"air_gap": LaunchConfiguration('air_gap')}
        ]
    )

    # Use SmartNeedle interface
    smart_needle_interface = Node(
        package = "trajcontrol",
        executable = "smart_needle_interface",
        parameters = [
            {"use_slicer": False},
        ]
    )

    # Use estimator node to calculate Jacobian
    estimator = Node(
        package = "trajcontrol",
        executable = "estimator",
        parameters = [
            {"save_J": True},
        ]
    )

    # Use manual controller
    # to be replaced by controller_rand
    rand_control = Node(
        package = "trajcontrol",
        executable = "controller_rand",
        parameters = [
            {"insertion_step": LaunchConfiguration('insertion_step')},
        ]
    )

    # # Save data to filename defined by user
    # save_file = Node(
    #     package = "trajcontrol",
    #     executable = "save_file",
    #     parameters =[{"filename": LaunchConfiguration('filename')}]
    # )

    # Include launch arguments
    ld.add_action(arg_insertion_step)
    ld.add_action(arg_air_gap)
    
    ld.add_action(rand_control)
    ld.add_action(estimator)
    ld.add_action(smart_needle_interface)
    ld.add_action(planning)
    
    return ld
