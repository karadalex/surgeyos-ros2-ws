# Foxglove Bridge

Optional ROS2 event streaming to Foxglove.

## Install and Run

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

## Connect

In Foxglove app, connect to:

```text
ws://<RASPBERRY_PI_IP>:8765
```
