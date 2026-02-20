# Video Streaming

Stream camera feeds to another machine on the same network without requiring ROS2 on the client.

## Install Components

```bash
sudo apt install ros-${ROS_DISTRO}-web-video-server
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 run web_video_server web_video_server
```

Install camera package:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt install ros-$ROS_DISTRO-camera-ros
```

## Run Pipeline

Terminal 1 (camera):

```bash
ros2 run camera_ros camera_node
```

Terminal 2 (web video server):

```bash
ros2 run web_video_server web_video_server
```

Terminal 3 (vision):

```bash
cd surgeyos-ros2-ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select vision
source install/setup.bash
ros2 launch vision mounting_detection.launch.py
```

## Access Stream

Open in browser on local network:

```text
http://<RASPBERRY_PI_IP>:8080
```
