SurgeyOS ROS2 Workspace
========================


## Instructions

(robot simulation still not working)

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

### To prepare the raspberry pi:

1. Install ubuntu server using raspberry pi imager with username, password and wifi configurations
2. Login to raspberry pi (preferrable via vscode to easily upload files)
3. upload install_ros2_rolling.sh from scripts directory
4. Run 
```bash
chmod +x install_ros2_rolling.sh
./install_ros2_rolling.sh
```
5. Install ubuntu desktop 
```bash
sudo apt install -y ubuntu-desktop
```
