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

# Launch stage control in mcp mode
# Remember to launch PlusServer connected to Aurora in another terminal
# Remember to run keypress node (trajcontrol package) in another terminal

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

    # Use needle.launch.py for the needle (sim_level = 1, IP = default, needleParamFile = default)
    needle = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('trajcontrol'), 'launch', 'jhu_needle.launch.py')
            ),
            launch_arguments = {
                'sim_level': '2',
                'interrogatorIP' : '10.0.0.55',
                'needleParamFile': 'needle_params_2021-08-16_Jig-Calibration_best.json',
            }.items()
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
        executable = "controller_mpc",
        parameters = [
            {"insertion_length": -100.0},
            {"H": LaunchConfiguration('H')},
            {"filename": LaunchConfiguration('filename')}
            ]
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
            "H",
            default_value = "4",
            description = "MPC horizon size"
        ),
        actions.LogInfo(msg = ["H: ", LaunchConfiguration('H')]),
        robot,
        needle,
        sensor,
        estimator,
        controller,
        save_file,
    ])
