#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Arm1DemoJointStates(Node):
    """Publishes gentle sinusoidal motion for the arm1 revolute joints."""

    def __init__(self) -> None:
        super().__init__("arm1_demo_joint_states")

        self.declare_parameter("output_topic", "/arm1_joint_states")
        self.declare_parameter("rate_hz", 40.0)
        self.declare_parameter("frequency_hz", 0.15)
        self.declare_parameter("max_velocity_rad_s", 0.6)
        self.declare_parameter("amp_joint0", 0.0)
        self.declare_parameter("amp_joint1", 0.25)
        self.declare_parameter("amp_joint2", 0.20)
        self.declare_parameter("amp_joint3", 0.18)
        self.declare_parameter("amp_joint4", 0.15)
        self.declare_parameter("amp_joint5", 0.12)

        self._joint_names = [
            "arm1_joint0",
            "arm1_joint1",
            "arm1_joint2",
            "arm1_joint3",
            "arm1_joint4",
            "arm1_joint5",
        ]
        self._phase_offsets = [0.0, 0.0, 0.7, 1.4, 2.1, 2.8]

        self._pub = self.create_publisher(
            JointState, str(self.get_parameter("output_topic").value), 10
        )
        self._t0 = time.monotonic()
        self._last_tick = self._t0
        self._system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._last_positions = [0.0] * len(self._joint_names)

        rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(
            "Publishing arm1 demo motion on /arm1_joint_states for arm1_joint0..arm1_joint5."
        )

    def _on_timer(self) -> None:
        now = time.monotonic()
        t = now - self._t0
        dt = max(now - self._last_tick, 1e-6)
        self._last_tick = now

        omega_t = 2.0 * math.pi * float(self.get_parameter("frequency_hz").value) * t
        amplitudes = [
            float(self.get_parameter("amp_joint0").value),
            float(self.get_parameter("amp_joint1").value),
            float(self.get_parameter("amp_joint2").value),
            float(self.get_parameter("amp_joint3").value),
            float(self.get_parameter("amp_joint4").value),
            float(self.get_parameter("amp_joint5").value),
        ]

        desired = [
            amplitude * math.sin(omega_t + phase)
            for amplitude, phase in zip(amplitudes, self._phase_offsets)
        ]

        max_vel = max(float(self.get_parameter("max_velocity_rad_s").value), 1e-6)
        max_step = max_vel * dt
        smoothed = []
        for prev, target in zip(self._last_positions, desired):
            delta = target - prev
            if delta > max_step:
                cur = prev + max_step
            elif delta < -max_step:
                cur = prev - max_step
            else:
                cur = target
            smoothed.append(cur)
        self._last_positions = smoothed

        msg = JointState()
        msg.header.stamp = self._system_clock.now().to_msg()
        msg.header.frame_id = ""
        msg.name = self._joint_names
        msg.position = smoothed
        msg.velocity = []
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Arm1DemoJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
