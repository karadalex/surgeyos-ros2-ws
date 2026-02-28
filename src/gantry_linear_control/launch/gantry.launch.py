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
        Node(
            package="gantry_linear_control",
            executable="xyz_path_planner",
            name="xyz_path_planner",
            output="screen",
            parameters=[
                {
                    "segment_length_mm": 25.0,
                    "default_feed_mm_s": 80.0,
                    "x_min_mm": -1000.0,
                    "x_max_mm": 1000.0,
                    "y_min_mm": -1000.0,
                    "y_max_mm": 1000.0,
                    "z_min_mm": -1000.0,
                    "z_max_mm": 1000.0,
                }
            ],
        ),
    ])
