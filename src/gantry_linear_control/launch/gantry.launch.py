from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gantry_linear_control",
            executable="serial_driver_node",
            name="serial_driver_node",
            output="screen",
            parameters=[{"port": "/dev/ttyUSB0", "baud": 115200}],
        ),
        Node(
            package="gantry_linear_control",
            executable="linear_action_server",
            name="linear_action_server",
            output="screen",
            parameters=[{"port": "/dev/ttyUSB0", "baud": 115200}],
        ),
    ])
