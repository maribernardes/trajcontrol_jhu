import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('trajcontrol'),
        'config',
        'virtual_nodes_params.yaml'
        )

    sensor = Node(
        package="trajcontrol",
        executable="virtual_sensor",
        parameters=[config]
    )

    robot = Node(
        package="trajcontrol",
        executable="virtual_robot",
        parameters=[config]
    )

    estimator = Node(
        package="trajcontrol",
        executable="estimator_node",
        parameters=[config]
    )

    controller = Node(
        package="trajcontrol",
        executable="controller_node"
    )   

    user = Node(
        package="trajcontrol",
        executable="virtual_UI"
    )

    ld.add_action(sensor)
    ld.add_action(robot)
    ld.add_action(estimator)
    #ld.add_action(controller)
    #ld.add_action(user)

    return ld