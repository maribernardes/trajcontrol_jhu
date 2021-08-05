from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sensor = Node(
        package="trajcontrol",
        executable="virtual_sensor",
    )

    robot = Node(
        package="trajcontrol",
        executable="virtual_robot"
    )

    user = Node(
        package="trajcontrol",
        executable="virtual_UI"
    )

    estimator = Node(
        package="trajcontrol",
        executable="estimator_node"
    )

    controller = Node(
        package="trajcontrol",
        executable="controller_node"
    )        
    ld.add_action(sensor)
    ld.add_action(robot)
    ld.add_action(user)
    ld.add_action(estimator)
    ld.add_action(controller)

    return ld