#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DualArmContinuousJointStates(Node):
    """Publishes continuous joint trajectory for two ROARM chains."""

    def __init__(self):
        super().__init__("dual_arm_continuous_joint_states")

        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("rate_hz", 40.0)
        self.declare_parameter("frequency_hz", 0.20)
        self.declare_parameter("amp_base", 0.35)
        self.declare_parameter("amp_l1_l2", 0.45)
        self.declare_parameter("amp_l2_l3", 0.45)
        self.declare_parameter("amp_l3_l4", 0.30)
        self.declare_parameter("amp_l4_l5", 0.20)
        self.declare_parameter("amp_l5_l53", 0.20)
        self.declare_parameter("max_velocity_rad_s", 0.80)

        self._joint_names = [
            "roarm1_base_to_L1",
            "roarm1_L1_to_L2",
            "roarm1_L2_to_L3",
            "roarm1_L3_to_L4",
            "roarm1_L4_to_L5_1_A",
            "roarm1_L4_to_L5_1_B",
            "roarm1_L4_to_L5_2_A",
            "roarm1_L4_to_L5_2_B",
            "roarm1_L5_1_A_to_L5_3_A",
            "roarm1_L5_1_B_to_L5_3_B",
            "roarm2_base_to_L1",
            "roarm2_L1_to_L2",
            "roarm2_L2_to_L3",
            "roarm2_L3_to_L4",
            "roarm2_L4_to_L5_1_A",
            "roarm2_L4_to_L5_1_B",
            "roarm2_L4_to_L5_2_A",
            "roarm2_L4_to_L5_2_B",
            "roarm2_L5_1_A_to_L5_3_A",
            "roarm2_L5_1_B_to_L5_3_B",
        ]

        self._pub = self.create_publisher(
            JointState, str(self.get_parameter("output_topic").value), 10
        )
        self._t0 = time.monotonic()
        self._last_tick = self._t0
        self._system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._last_positions = [0.0] * len(self._joint_names)

        rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info("Publishing dual-arm continuous motion to /joint_states.")

    def _on_timer(self) -> None:
        now = time.monotonic()
        t = now - self._t0
        dt = max(now - self._last_tick, 1e-6)
        self._last_tick = now
        f = float(self.get_parameter("frequency_hz").value)
        wt = 2.0 * math.pi * f * t

        a0 = float(self.get_parameter("amp_base").value)
        a1 = float(self.get_parameter("amp_l1_l2").value)
        a2 = float(self.get_parameter("amp_l2_l3").value)
        a3 = float(self.get_parameter("amp_l3_l4").value)
        a4 = float(self.get_parameter("amp_l4_l5").value)
        a5 = float(self.get_parameter("amp_l5_l53").value)

        # Arm 1
        arm1 = [
            a0 * math.sin(wt),
            a1 * math.sin(wt + 0.6),
            a2 * math.sin(wt + math.pi),
            a3 * math.sin(wt + 1.2),
            a4 * math.sin(wt + 0.8),
            -a4 * math.sin(wt + 0.8),
            a4 * math.sin(wt + 0.8),
            -a4 * math.sin(wt + 0.8),
            a5 * math.sin(wt + 1.4),
            -a5 * math.sin(wt + 1.4),
        ]

        # Arm 2 mirrored phase
        arm2 = [
            a0 * math.sin(wt + math.pi),
            a1 * math.sin(wt + math.pi + 0.6),
            a2 * math.sin(wt + 2.0 * math.pi),
            a3 * math.sin(wt + math.pi + 1.2),
            a4 * math.sin(wt + math.pi + 0.8),
            -a4 * math.sin(wt + math.pi + 0.8),
            a4 * math.sin(wt + math.pi + 0.8),
            -a4 * math.sin(wt + math.pi + 0.8),
            a5 * math.sin(wt + math.pi + 1.4),
            -a5 * math.sin(wt + math.pi + 1.4),
        ]

        desired = arm1 + arm2
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
    node = DualArmContinuousJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
