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

6. Install a VNC server you control (recommended for predictable VNC): TigerVNC

This creates a separate VNC desktop session you can reach from your Mac.

7. Install TigerVNC + a lightweight desktop for the VNC session

(Using a light window manager helps performance and avoids GNOME/Wayland quirks.)
```bash
sudo apt update
sudo apt install -y tigervnc-standalone-server tigervnc-common openbox
```

This “TigerVNC + Openbox” pattern is commonly used on Ubuntu 24.04-ish setups.  ￼

8. Set a VNC password and create initial config

```bash
vncserver
vncserver --kill :1
```

9. Set the VNC session to start a desktop (Openbox example)

Edit:
```bash
nano ~/.vnc/xstartup
```

Put something like:

```bash
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec openbox-session
```

Then:
```bash
chmod +x ~/.vnc/xstartup
```

10. Start the VNC server (secure-ish defaults)

This starts display :1 which maps to TCP port 5901.

```bash
vncserver :1 -geometry 1920x1080 -depth 24 -localhost
```

-localhost is important: it prevents direct network access so you can tunnel over SSH instead.  ￼

11.  Connect from macOS

macOS has a built-in VNC client via Screen Sharing / Finder “Connect to Server”.

Best practice (secure): SSH tunnel + connect to localhost
	a.	From your Mac, create the tunnel:
```bash
ssh -L 5901:localhost:5901 <ubuntu-user>@<pi-ip>
```
b.	Then on your Mac:

Finder → Go → Connect to Server… Enter: vnc://localhost:5901

This “vnc://…” method is the standard macOS way to connect.  ￼

Less secure (LAN only): open the port directly

If you don’t use -localhost, you can connect to:
vnc://<pi-ip>:5901

If you do that, at least firewall it:
```bash
sudo ufw allow 5901/tcp
```