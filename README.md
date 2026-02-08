SurgeyOS ROS2 Workspace
========================


## Instructions

```bash
source /opt/ros/${ROS_DISTRO}/setup.sh
python3 -m venv .venv
pip install -r requirements.txt
sudo apt install -y ros-${ROS_DISTRO}-cv-bridge python3-opencv
sudo apt install ros-rolling-xacro
sudo apt install ros-rolling-joint-state-publisher ros-rolling-robot-state-publisher ros-rolling-rviz2
colcon build --symlink-install
source install/setup.bash
ros2 run vision mounting_detection
ros2 launch gantry_linear_control gantry.launch.py
```
in caser there are issues with the rviz:
```bash
sudo apt full-upgrade -y
sudo apt install --reinstall -y \
  ros-rolling-ros-base \
  ros-rolling-rcl \
  ros-rolling-rclpy \
  ros-rolling-rmw-implementation \
  ros-rolling-rmw-fastrtps-cpp \
  ros-rolling-rmw-fastrtps-shared-cpp \
  ros-rolling-rcutils \
  ros-rolling-rclcpp
```

To run the simulation:
```bash
ros2 launch robot_description display.launch.py use_rviz:=true
```