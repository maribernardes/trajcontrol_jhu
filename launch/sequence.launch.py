import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument

# Launch stage control in manual mode
# Remember to launch PlusServer connected to Aurora in another terminal

def generate_launch_description():

    robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('stage_control'), 'launch', 'stage_control_launch.py')
            ),
            launch_arguments={
                'sim_level': '2' 
            }.items()
    )
    
    aurora = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        parameters=[
            {"RIB_server_ip":"localhost"},
            {"RIB_port": 18944},
            {"RIB_type": "client"}
        ]
    )

    sensor = Node(
        package = "trajcontrol",
        executable = "sensor_processing"
    )

    controller = Node(
        package="trajcontrol",
        executable="controller_sequence",
    )   

    save_file = Node(
        package="trajcontrol",
        executable="save_file",
        parameters=[{"filename":LaunchConfiguration('filename')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "filename",
            default_value="my_data",
            description="File name to save .csv file with experimental data"
        ),
        actions.LogInfo(msg=["filename: ", LaunchConfiguration('filename')]),
        robot,
        aurora,
        sensor,
        controller,
        save_file,
    ])
