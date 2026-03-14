from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='detection',
            name='detection',
            output='screen',
            parameters=[
                {'input_topic': '/camera/image_raw'},
                {'output_topic': '/image/processed'},
                {'min_hole_area_px': 60.0},
            ],
        ),
    ])
