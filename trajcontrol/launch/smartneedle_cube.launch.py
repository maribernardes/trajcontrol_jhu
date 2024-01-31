import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument, ExecuteProcess
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node

# Launch trajcontrol environment for experiment (with 3D Slicer)

def generate_launch_description():

    ld = LaunchDescription()

    arg_cube_size = DeclareLaunchArgument(
            "cube_size",
            default_value = "50.0",
            description = "Size of cube in mm"
    )

    arg_fiducial_offset = DeclareLaunchArgument(
            "fiducial_offset",
            default_value = "20.0",
            description = "Fiducial offset from the guide"
    )

    arg_filename = DeclareLaunchArgument(
            "filename",
            default_value = "cube",
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

    # Use cube controller
    cube_control = Node(
        package = "trajcontrol",
        executable = "controller_cube",
        emulate_tty = True,
        parameters = [
            {"cube_size": LaunchConfiguration('cube_size')},
            {"fiducial_offset": LaunchConfiguration('fiducial_offset')},
            {"filename": LaunchConfiguration('filename')},
        ]
    )

    rosbag = ExecuteProcess(
        cmd = ["ros2", "bag", "record", "-a", "-o", LaunchConfiguration('ros_filename')],
        output = 'screen'
    )

    # Include launch arguments
    ld.add_action(arg_cube_size)
    ld.add_action(arg_fiducial_offset)
    ld.add_action(arg_filename)
    ld.add_action(arg_ros_filename)
    
    ld.add_action(cube_control)
    ld.add_action(planning)

    ld.add_action(rosbag)
    
    return ld
