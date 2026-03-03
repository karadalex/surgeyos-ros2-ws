from setuptools import find_packages, setup

package_name = 'arm_path'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karadalex',
    maintainer_email='karadalex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_wave = arm_path.path_planner_wave:main',
            'keyboard_teleop = arm_path.keyboard_teleop:main',
            'keyboard_joint_teleop = arm_path.keyboard_joint_teleop:main',
            'dual_arm_continuous_joint_states = arm_path.dual_arm_continuous_joint_states:main',
            'arm1_demo_joint_states = arm_path.arm1_demo_joint_states:main',
            'gazebo_demo_joint_commands = arm_path.gazebo_demo_joint_commands:main',
        ],
    },
)
