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

    # Use smart template node
    robot = Node(
        package="smart_template",
        executable="template",
    )  

    igtl_bridge = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        parameters=[
            {"RIB_server_ip":"localhost"},
            {"RIB_port": 18944},
            {"RIB_type": "client"}
        ]
    )

    # If commented, launch needle separetly (good for debugging)
    # # Use needle.launch.py for the needle (sim_level = 1, IP = default, needleParamFile = default)
    # needle = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory('trajcontrol'), 'launch', 'jhu_needle.launch.py')
    #         ),
    #         launch_arguments = {
    #             'sim_level': '2',
    #             'interrogatorIP' : '10.0.0.55',
    #             'needleParamFile': '3CH-4AA-0005_needle_params_2022-01-26_Jig-Calibration_best_weights.json',
    #         }.items()
    # )

    # Get depth measurement
    depth = Node(
        package = "trajcontrol",
        executable = "virtual_depth_measurement",
    )

    # Use sensor processing node with final insertion length of 100mm
    demo = Node(
        package = "trajcontrol",
        executable = "trajcontrol_demo_step"
    )


    # Use system interface node with final insertion length of 100mm
    interface = Node(
        package = "trajcontrol",
        executable = "system_interface",
        parameters = [
            {"insertion_length": 100.0}
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
        robot,
        igtl_bridge,
        # needle,
        depth,
        interface,
        demo,
        save_file,
    ])
