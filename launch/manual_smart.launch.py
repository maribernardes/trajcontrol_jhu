import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Launch SmartTemplate in manual control mode
# Remember to run keypress node (trajcontrol package) in another terminal

def generate_launch_description():

    ld = LaunchDescription()

    arg_sim_level = DeclareLaunchArgument(
        "sim_level",
        default_value = "2",
        description = "virtual = 1, real = 2"
    )  
    arg_lateral_step = DeclareLaunchArgument(
            "lateral_step",
            default_value = "1.0",
            description = "Lateral step size in mm"
    )
    arg_insertion_step = DeclareLaunchArgument(
            "insertion_step",
            default_value = "10.0",
            description = "Insertion step size in mm"
    )

    launch_directory = os.path.join(get_package_share_directory('smart_template'), 'launch')
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_directory, 'robot.launch.py')),
        condition=conditions.IfCondition(
            PythonExpression([LaunchConfiguration('sim_level'), " == 2"])
        )
    )
    virtual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_directory, 'virtual_robot.launch.py')),
        condition=conditions.IfCondition(
            PythonExpression([LaunchConfiguration('sim_level'), " == 1"])
        )
    )

    # Use manual controller
    manual_control = Node(
        package = "trajcontrol",
        executable = "controller_manual_smart",
        parameters = [
            {"lateral_step": LaunchConfiguration('lateral_step')},
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
    ld.add_action(arg_sim_level)
    ld.add_action(arg_lateral_step)
    ld.add_action(arg_insertion_step)

    ld.add_action(hardware_launch)
    ld.add_action(virtual_launch)
    ld.add_action(manual_control)
    
    return ld