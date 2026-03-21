from glob import glob
import os

from setuptools import find_packages, setup


package_name = "gantry_controller"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Alexios Karadimos",
    maintainer_email="karadalex@gmail.com",
    description=(
        "ROS 2 gantry controller that consumes Point targets "
        "and drives a serial gantry MCU."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gantry_controller_node = gantry_controller.controller_node:main",
        ],
    },
)
