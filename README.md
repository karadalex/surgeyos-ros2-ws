SurgeyOS ROS2 Workspace
========================


## Instructions

```bash
source /opt/ros/${ROS_DISTRO}/setup.sh
python3 -m venv .venv
pip install -r requirements.txt
sudo apt install -y ros-${ROS_DISTRO}-cv-bridge python3-opencv
sudo apt install ros-rolling-xacro
colcon build --symlink-install
source install/setup.bash
ros2 run vision mounting_detection
ros2 launch gantry_linear_control gantry.launch.py
```

or for the simulation
```bash
ros2 launch robot_description display.launch.py
```