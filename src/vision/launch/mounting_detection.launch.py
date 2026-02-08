from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='mounting_detection',
            name='mounting_detection',
            output='screen',
            parameters=[
                {'input_topic': '/image'},
                {'output_topic': '/image/processed'},
                {'use_canny': True},
            ],
        ),
    ])
