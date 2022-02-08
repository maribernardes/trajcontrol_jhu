import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('trajcontrol'),
        'config',
        'demo_params.yaml'
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

    file = Node(
        package="trajcontrol",
        executable="save_file",
        parameters=[config]
    )

    ld.add_action(estimator)
    ld.add_action(controller)
    ld.add_action(user)
    ld.add_action(file)

    return ld