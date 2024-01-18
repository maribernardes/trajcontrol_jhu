import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument, ExecuteProcess
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

    arg_num_blocks = DeclareLaunchArgument(
            "num_blocks",
            default_value = "0",
            description = "Number of LEGO blocks"
    )

    arg_filename = DeclareLaunchArgument(
            "filename",
            default_value = "sequence",
            description = "Data logs filename prefix"
    )

    arg_ros_filename = DeclareLaunchArgument(
            'ros_filename', 
            default_value = [LaunchConfiguration('filename'), '_'+datetime.now().strftime("%Y_%m_%d-%H_%M_%S")]
    )

    # Use planning interface
    # air_gap and insertion_length are not used because use_slicer is True
    planning= Node(
        package = "trajcontrol",
        executable = "planning",
        emulate_tty = True,
        parameters = [
            {"use_slicer": True},
        ]
    )

    # Use SmartNeedle interface
    smart_needle_interface = Node(
        package = "trajcontrol",
        executable = "smart_needle_interface",
        parameters = [
            {"use_slicer": True},
            {"num_blocks": LaunchConfiguration('num_blocks')}
        ]
    )

    # Use open-loop controller
    # to be replaced by controller_openloop
    open_control = Node(
        package = "trajcontrol",
        executable = "controller_openloop",
        emulate_tty = True,
        parameters = [
            {"lateral_step": 1.0},
            {"insertion_step": LaunchConfiguration('insertion_step')},
            {"filename": LaunchConfiguration('filename')},
        ]
    )

    rosbag = ExecuteProcess(
        cmd = ["ros2", "bag", "record", "-a", "-o", LaunchConfiguration('ros_filename')],
        output = 'screen'
    )

    # Include launch arguments
    ld.add_action(arg_insertion_step)
    ld.add_action(arg_num_blocks)
    ld.add_action(arg_filename)
    ld.add_action(arg_ros_filename)
    
    ld.add_action(open_control)
    ld.add_action(planning)
    ld.add_action(smart_needle_interface)

    ld.add_action(rosbag)
    
    return ld
