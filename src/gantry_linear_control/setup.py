from setuptools import find_packages, setup

package_name = "gantry_linear_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/gantry.launch.py"]),
        ("share/" + package_name + "/msg", ["msg/GantryState.msg"]),
        ("share/" + package_name + "/action", ["action/MoveLinear.action"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="ROS 2 Python control for a 3-axis Cartesian gantry via NodeMCU over serial.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serial_driver_node = nodes.serial_driver_node:main",
            "linear_action_server = nodes.linear_action_server:main",
            "xyz_path_planner = nodes.xyz_path_planner:main",
        ],
    },
)
