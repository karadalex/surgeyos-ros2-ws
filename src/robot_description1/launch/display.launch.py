from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_rviz = LaunchConfiguration("use_rviz")
    use_software_gl = LaunchConfiguration("use_software_gl")
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_web_video_server = LaunchConfiguration("use_web_video_server", default="true")

    return LaunchDescription([
        DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_software_gl", default_value="true"),
        DeclareLaunchArgument("use_web_video_server", default_value="true"),
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution([
                FindPackageShare("robot_description1"),
                "urdf",
                "robot.urdf.xacro",
            ]),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("robot_description1"),
                "rviz",
                "view.rviz",
            ]),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command([FindExecutable(name="xacro"), " ", model]),
                    value_type=str,
                ),
            }],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=IfCondition(use_joint_state_publisher),
            output="screen",
            parameters=[{
                # Merge externally-published joint states so animated joints
                # appear in RViz while unspecified joints still get defaults.
                "source_list": ["/gantry_joint_states", "/arm1_joint_states"],
            }],
        ),
        Node(
            package="web_video_server",
            executable="web_video_server",
            condition=IfCondition(use_web_video_server),
            output="screen",
        ),
        SetEnvironmentVariable(
            name="LIBGL_ALWAYS_SOFTWARE",
            value="1",
            condition=IfCondition(use_software_gl),
        ),
        SetEnvironmentVariable(
            name="QT_XCB_GL_INTEGRATION",
            value="none",
            condition=IfCondition(use_software_gl),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
