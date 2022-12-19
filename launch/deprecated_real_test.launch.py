import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    estimator = Node(
        package = "trajcontrol",
        executable = "estimator_node",
    )

    controller = Node(
        package="trajcontrol",
        executable="controller_node"
    )   

    savefile = Node(
        package = "trajcontrol",
        executable = "save_file",
        parameters=[{"filename":LaunchConfiguration('filename')}]       
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "registration",
            default_value="0",
            description="0=load previous / 1=new registration"
        ),
        actions.LogInfo(msg=["registration: ", LaunchConfiguration('registration')]),
        actions.LogInfo(msg=["registration: ", LaunchConfiguration('registration')]),
        DeclareLaunchArgument(
            "filename",
            default_value="my_data",
            description="Name for .cvs file with experimental data"
        ),
        actions.LogInfo(msg=["filename: ", LaunchConfiguration('filename')]),
        savefile,
        estimator,
        controller,
    ])
