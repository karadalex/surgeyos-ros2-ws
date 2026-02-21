from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    use_robot_description2_display = LaunchConfiguration("use_robot_description2_display", default="false")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="")
    use_software_gl = LaunchConfiguration("use_software_gl", default="true")
    use_web_video_server = LaunchConfiguration("use_web_video_server", default="true")

    robot_description2_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_description2"), "launch", "display.launch.py"]
            )
        ),
        launch_arguments={
            "use_joint_state_publisher": use_joint_state_publisher,
            "use_rviz": use_rviz,
            "use_software_gl": use_software_gl,
        }.items(),
        condition=IfCondition(use_robot_description2_display)
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_robot_description2_display", default_value="false"),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_software_gl", default_value="true"),
            DeclareLaunchArgument("use_web_video_server", default_value="true"),
            robot_description2_display,
            Node(
                package="camera_ros",
                executable="camera_node",
                output="screen",
            ),
            Node(
                package="vision",
                executable="mounting_detection",
                output="screen",
            ),
            Node(
                package="web_video_server",
                executable="web_video_server",
                condition=IfCondition(use_web_video_server),
                output="screen",
            ),
            Node(
                package="serial_ctrl",
                executable="serial_ctrl_py",
                output="screen",
            ),
            Node(
                package="kinematics",
                executable="tf_to_inverse",
                output="screen",
            ),
        ]
    )
