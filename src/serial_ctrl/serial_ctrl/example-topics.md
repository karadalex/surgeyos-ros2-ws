Home position
```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['j1','j2','j3','j4','j5'], position: [0.0, 0.0, 0.0, 0.0, 0.0], velocity: [], effort: []}"
```

send continuously
```bash
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['j1','j2','j3','j4','j5'], position: [0.0, 0.5, -0.3, 0.2, 0.1], velocity: [], effort: []}"
```

Close gripper
```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['j1','j2','j3','j4','j5'], position: [0.0, 0.1, 0.0, 0.0, -3.14], velocity: [], effort: []}"
```

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['j1','j2','j3','j4','j5'], position: [-3.14, 1.5, -1.5, 0.0, -3.14], velocity: [], effort: []}"
```