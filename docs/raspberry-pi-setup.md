# Raspberry Pi Setup

## Base OS and ROS2

1. Install Ubuntu Server via Raspberry Pi Imager (set username, password, Wi-Fi).
2. Log in to the Raspberry Pi (VS Code remote access is convenient).
3. Upload `scripts/install_ros2_rolling.sh`.
4. Run:

```bash
chmod +x install_ros2_rolling.sh
./install_ros2_rolling.sh
```

5. Install Ubuntu Desktop:

```bash
sudo apt install -y ubuntu-desktop
```

## VNC Setup (TigerVNC + Openbox)

6. Install TigerVNC and Openbox:

```bash
sudo apt update
sudo apt install -y tigervnc-standalone-server tigervnc-common openbox
```

7. Create initial VNC config:

```bash
vncserver
vncserver --kill :1
```

8. Configure startup script:

```bash
nano ~/.vnc/xstartup
```

Use:

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

9. Start VNC server on display `:1`:

```bash
vncserver :1 -geometry 1920x1080 -depth 24 -localhost
```

## Connect from macOS

Recommended: SSH tunnel.

```bash
ssh -L 5901:localhost:5901 <ubuntu-user>@<pi-ip>
```

Then connect from Finder:

```text
vnc://localhost:5901
```

Less secure (LAN):

```text
vnc://<pi-ip>:5901
```

If opened directly, at least firewall the port:

```bash
sudo ufw allow 5901/tcp
```
