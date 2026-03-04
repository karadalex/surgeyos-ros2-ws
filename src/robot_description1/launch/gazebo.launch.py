import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _resource_paths():
    launch_dir = Path(__file__).resolve().parent
    robot_source_dir = launch_dir.parent
    workspace_src_dir = robot_source_dir.parent
    objects_source_dir = workspace_src_dir / "objects"

    paths = [
        str(robot_source_dir),
        str(workspace_src_dir),
    ]

    if objects_source_dir.exists():
        paths.append(str(objects_source_dir))

    for package_name in ("robot_description1", "objects"):
        try:
            share_dir = Path(get_package_share_directory(package_name)).resolve()
        except PackageNotFoundError:
            continue

        paths.extend([str(share_dir.parent), str(share_dir)])

    unique_paths = []
    for path in paths:
        if path and path not in unique_paths:
            unique_paths.append(path)

    return unique_paths


def _resource_path_value(variable_name):
    value = []
    for path in _resource_paths():
        value.extend([path, os.pathsep])
    value.append(EnvironmentVariable(variable_name, default_value=""))
    return value


def generate_launch_description():
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_rviz = LaunchConfiguration("use_rviz")
    use_demo_controller = LaunchConfiguration("use_demo_controller")
    robot_file = LaunchConfiguration("robot_file", default="robot.urdf.xacro")
    rviz_config = LaunchConfiguration("rviz_config")
    generated_urdf = "/tmp/robot_description1_gazebo.urdf"
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("robot_description1"), "config", "gazebo_bridge.yaml"]
    )

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
                    [FindPackageShare("robot_description1"), "urdf", robot_file]
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_description1"), "worlds", "empty.sdf"]
                ),
            ),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_demo_controller", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_description1"), "rviz", "view.rviz"]
                ),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=_resource_path_value("GZ_SIM_RESOURCE_PATH"),
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=_resource_path_value("IGN_GAZEBO_RESOURCE_PATH"),
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
                parameters=[{"config_file": bridge_config}],
            ),
            Node(
                package="arm_path",
                executable="gazebo_demo_joint_commands",
                output="screen",
                condition=IfCondition(use_demo_controller),
                parameters=[{"use_sim_time": True}],
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
