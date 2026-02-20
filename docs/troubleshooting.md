# Troubleshooting

## RViz/ROS Runtime Repair

If RViz or core ROS2 packages are in a bad state:

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
