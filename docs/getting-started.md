# Getting Started

## Prepare Python Environment

```bash
python3 -m venv .venv
sudo apt install -y python3-numpy
pip install -r requirements.txt
python -m pip install -U numpy
source /home/robot/surgeyos-ros2-ws/.venv/bin/activate
python -m pip install -U pip setuptools wheel
python -m pip install -U catkin_pkg lark empy pyyaml packaging
```

## Install ROS2 Dependencies and Build

```bash
source /opt/ros/${ROS_DISTRO}/setup.sh
sudo apt install -y ros-${ROS_DISTRO}-cv-bridge python3-opencv
sudo apt install ros-rolling-xacro
sudo apt install ros-rolling-joint-state-publisher ros-rolling-robot-state-publisher ros-rolling-rviz2
colcon build --symlink-install
source install/setup.bash
```

## Run Main Nodes

```bash
ros2 run vision mounting_detection
ros2 launch gantry_linear_control gantry.launch.py
```
