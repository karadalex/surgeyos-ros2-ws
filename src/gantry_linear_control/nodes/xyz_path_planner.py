#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState


class XYZPathPlanner(Node):
    """Publishes simple continuous motion for 3 gantry joints."""

    def __init__(self):
        super().__init__("xyz_path_planner")

        self._joint_names = [
            "extra_root_2_to_gantry_column",
            "extra_root_1_to_axis1",
            "z",
        ]

        self.declare_parameter(
            "joint_names",
            self._joint_names,
        )
        self.declare_parameter("output_topic", "/gantry_joint_states")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("amplitude_x_m", 0.20)
        self.declare_parameter("amplitude_y_m", 0.15)
        self.declare_parameter("amplitude_z_m", 0.10)
        self.declare_parameter("frequency_hz", 0.20)
        self.declare_parameter("phase_y_rad", math.pi / 2.0)
        self.declare_parameter("phase_z_rad", math.pi)

        output_topic = str(self.get_parameter("output_topic").value)
        self._joint_pub = self.create_publisher(JointState, output_topic, 10)
        self._t0_monotonic = time.monotonic()
        self._system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        publish_hz = max(float(self.get_parameter("publish_hz").value), 1e-3)
        self._timer = self.create_timer(1.0 / publish_hz, self._publish_motion)
        self.get_logger().info(
            f"Publishing gantry joints on {output_topic}; joint_state_publisher should merge this into /joint_states."
        )

    def _publish_motion(self) -> None:
        elapsed = time.monotonic() - self._t0_monotonic
        freq = float(self.get_parameter("frequency_hz").value)
        omega_t = 2.0 * math.pi * freq * elapsed

        ax = float(self.get_parameter("amplitude_x_m").value)
        ay = float(self.get_parameter("amplitude_y_m").value)
        az = float(self.get_parameter("amplitude_z_m").value)

        py = float(self.get_parameter("phase_y_rad").value)
        pz = float(self.get_parameter("phase_z_rad").value)
        joint_names = list(self.get_parameter("joint_names").value)
        positions = [0.0] * len(joint_names)
        animated_values = {
            "extra_root_2_to_gantry_column": ax * math.sin(omega_t),
            "extra_root_1_to_axis1": ay * math.sin(omega_t + py),
            "z": az * math.sin(omega_t + pz),
        }
        for i, name in enumerate(joint_names):
            if name in animated_values:
                positions[i] = animated_values[name]

        js = JointState()
        js.header.stamp = self._system_clock.now().to_msg()
        js.header.frame_id = ""
        js.name = joint_names
        js.position = positions
        self._joint_pub.publish(js)


def main():
    rclpy.init()
    node = XYZPathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
