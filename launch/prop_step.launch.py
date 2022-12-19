import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument

# Launch stage control in simple mode
# Remember to launch PlusServer connected to Aurora in another terminal
# Remember to launch keypress node (trajcontrol package) in another terminal

def generate_launch_description():

    # Use ros2_needle_guide_robot launch for the stage_control with stage_hardware_node (sim_level = 2)
    robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('stage_control'), 'launch', 'stage_control_launch.py')
            ),
            launch_arguments = {
                'sim_level': '2' 
            }.items()
    )

    # Use ros2_igtl_bridge for client with port 18944 and localhost IP
    aurora = Node(
        package = "ros2_igtl_bridge",
        executable = "igtl_node",
        parameters = [
            {"RIB_server_ip": "localhost"},
            {"RIB_port": 18944},
            {"RIB_type": "client"}
        ]
    )

    # Use sensor processing node with final insertion length of 100mm
    sensor = Node(
        package = "trajcontrol",
        executable = "sensor_processing",
        parameters = [
            {"insertion_length": -100.0}
            ]
    )

    # Use estimator with standard K
    estimator = Node(
        package="trajcontrol",
        executable="estimator",
        parameters = [
            {"save_J": False}
        ]    
    ) 
    
    # Use simple controller (K*J*e)
    controller = Node(
        package = "trajcontrol",
        executable = "controller_proportional",
        parameters =[{"K": LaunchConfiguration('K')}]
    )   

    # Save data to filename defined by user
    save_file = Node(
        package = "trajcontrol",
        executable = "save_file",
        parameters =[{"filename": LaunchConfiguration('filename')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "filename",
            default_value = "my_data",
            description = "File name to save .csv file with experimental data"
        ),
        actions.LogInfo(msg = ["filename: ", LaunchConfiguration('filename')]),
        DeclareLaunchArgument(
            "K",
            default_value = "0.5",
            description = "Control gain"
        ),
        actions.LogInfo(msg = ["K: ", LaunchConfiguration('K')]),
        robot,
        aurora,
        sensor,
        estimator,
        controller,
        save_file,
    ])
