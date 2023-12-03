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


# Launch stage control in manual mode
# Remember to launch keypress node (trajcontrol package) in another terminal


def generate_launch_description():

    robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('stage_control'), 'launch', 'stage_control_launch.py')
            ),
            launch_arguments={
                'sim_level': '2' 
            }.items()
    )
    
    controller = Node(
        package="trajcontrol",
        executable="controller_init",
    )   

    return LaunchDescription([
        controller,
        robot,
    ])
