from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="gantry_controller",
                executable="gantry_controller_node",
                name="gantry_controller",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/ttyUSB0",
                        "baud": 115200,
                        "default_feed_mm_s": 40.0,
                        "x_min_mm": 0.0,
                        "x_max_mm": 1500.0,
                        "y_min_mm": 0.0,
                        "y_max_mm": 1500.0,
                        "z_min_mm": 0.0,
                        "z_max_mm": 300.0,
                    }
                ],
            )
        ]
    )
