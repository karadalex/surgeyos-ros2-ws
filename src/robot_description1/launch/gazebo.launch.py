from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    generated_urdf = "/tmp/robot_description1_gazebo.urdf"

    robot_description_content = Command([FindExecutable(name="xacro"), " ", model])
    robot_description = ParameterValue(
        robot_description_content,
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    generate_urdf = ExecuteProcess(
        cmd=[
            FindExecutable(name="xacro"),
            model,
            "-o",
            generated_urdf,
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "surgery_robot",
            "-file",
            generated_urdf,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_description1"), "urdf", "robot.urdf.xacro"]
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_description1"), "worlds", "empty.sdf"]
                ),
            ),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_description1"), "rviz", "view.rviz"]
                ),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    PathJoinSubstitution([FindPackageShare("robot_description1"), ".."]),
                    ":",
                    PathJoinSubstitution([FindPackageShare("robot_description1")]),
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    PathJoinSubstitution([FindPackageShare("robot_description1"), ".."]),
                    ":",
                    PathJoinSubstitution([FindPackageShare("robot_description1")]),
                    ":",
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            gz_sim,
            generate_urdf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=generate_urdf,
                    on_exit=[spawn_robot],
                )
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "source_list": ["/gantry_joint_states", "/arm1_joint_states"],
                    }
                ],
                condition=IfCondition(use_joint_state_publisher),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/z_bottom_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/z_bottom_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
