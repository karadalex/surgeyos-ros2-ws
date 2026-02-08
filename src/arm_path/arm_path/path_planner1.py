#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class WaveJointStates(Node):
    """
    Publishes a smooth "wave" on /joint_states (JointState).
    serial_ctrl-style drivers only use msg.position[0..4] ordering. :contentReference[oaicite:3]{index=3}
    """

    def __init__(self):
        super().__init__('roarm_m1_wave_joint_states')

        # Parameters (easy to tweak from CLI)
        self.declare_parameter('rate_hz', 40.0)
        self.declare_parameter('amp_rad_base', 0.25)      # joint 1 amplitude (rad)
        self.declare_parameter('amp_rad_shoulder', 0.35)  # joint 2 amplitude (rad)
        self.declare_parameter('amp_rad_elbow', 0.35)     # joint 3 amplitude (rad)
        self.declare_parameter('amp_rad_wrist', 0.30)     # joint 5 amplitude (rad)

        self.declare_parameter('f_base_hz', 0.10)
        self.declare_parameter('f_arm_hz', 0.18)
        self.declare_parameter('f_wrist_hz', 0.22)

        # Joint names are optional for the Waveshare-style serial node (it uses position array indices),
        # but setting them helps visualization tools.
        self.declare_parameter('joint_names', [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5'
        ])

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / max(rate_hz, 1.0)
        self.t0 = self.get_clock().now()

        self.timer = self.create_timer(self.dt, self.on_timer)

    def on_timer(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        amp1 = float(self.get_parameter('amp_rad_base').value)
        amp2 = float(self.get_parameter('amp_rad_shoulder').value)
        amp3 = float(self.get_parameter('amp_rad_elbow').value)
        amp5 = float(self.get_parameter('amp_rad_wrist').value)

        f1 = float(self.get_parameter('f_base_hz').value)
        fA = float(self.get_parameter('f_arm_hz').value)
        f5 = float(self.get_parameter('f_wrist_hz').value)

        # Wave motion:
        j1 = amp1 * math.sin(2.0 * math.pi * f1 * t)
        j2 = amp2 * math.sin(2.0 * math.pi * fA * t)
        j3 = amp3 * math.sin(2.0 * math.pi * fA * t + math.pi)  # out of phase
        j4 = 0.0  # keep gripper steady (safer)
        j5 = amp5 * math.sin(2.0 * math.pi * f5 * t)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.get_parameter('joint_names').value)
        msg.position = [j1, j2, j3, j4, j5]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = WaveJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
