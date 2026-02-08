SurgeyOS ROS2 Workspace
========================


## Instructions

```bash
source /opt/ros/${ROS_DISTRO}/setup.sh
python3 -m venv .venv
pip install -r requirements.txt
sudo apt install -y ros-${ROS_DISTRO}-cv-bridge python3-opencv
colcon build --symlink-install
source install/setup.bash
ros2 run vision mounting_detection
```