#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GazeboDemoJointCommands(Node):
    """Publishes smooth position commands for the Gazebo gantry and arm1 joints."""

    def __init__(self) -> None:
        super().__init__("gazebo_demo_joint_commands")

        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("frequency_hz", 0.10)
        self.declare_parameter("amp_x_m", 0.15)
        self.declare_parameter("amp_y_m", 0.10)
        self.declare_parameter("amp_z_m", 0.08)
        self.declare_parameter("amp_arm1_joint1", 0.20)
        self.declare_parameter("amp_arm1_joint2", 0.18)
        self.declare_parameter("amp_arm1_joint3", 0.15)
        self.declare_parameter("amp_arm1_joint4", 0.12)
        self.declare_parameter("amp_arm1_joint5", 0.10)

        self._topic_by_joint = {
            "extra_root_2_to_gantry_column": "/sim/extra_root_2_to_gantry_column/cmd_pos",
            "extra_root_1_to_axis1": "/sim/extra_root_1_to_axis1/cmd_pos",
            "z": "/sim/z/cmd_pos",
            "arm1_joint1": "/sim/arm1_joint1/cmd_pos",
            "arm1_joint2": "/sim/arm1_joint2/cmd_pos",
            "arm1_joint3": "/sim/arm1_joint3/cmd_pos",
            "arm1_joint4": "/sim/arm1_joint4/cmd_pos",
            "arm1_joint5": "/sim/arm1_joint5/cmd_pos",
        }
        self._phase_by_joint = {
            "extra_root_2_to_gantry_column": 0.0,
            "extra_root_1_to_axis1": math.pi / 2.0,
            "z": math.pi,
            "arm1_joint1": 0.3,
            "arm1_joint2": 0.9,
            "arm1_joint3": 1.5,
            "arm1_joint4": 2.1,
            "arm1_joint5": 2.7,
        }
        self._amp_by_joint = {
            "extra_root_2_to_gantry_column": float(self.get_parameter("amp_x_m").value),
            "extra_root_1_to_axis1": float(self.get_parameter("amp_y_m").value),
            "z": float(self.get_parameter("amp_z_m").value),
            "arm1_joint1": float(self.get_parameter("amp_arm1_joint1").value),
            "arm1_joint2": float(self.get_parameter("amp_arm1_joint2").value),
            "arm1_joint3": float(self.get_parameter("amp_arm1_joint3").value),
            "arm1_joint4": float(self.get_parameter("amp_arm1_joint4").value),
            "arm1_joint5": float(self.get_parameter("amp_arm1_joint5").value),
        }

        self._publishers = {
            joint: self.create_publisher(Float64, topic, 10)
            for joint, topic in self._topic_by_joint.items()
        }
        self._t0 = time.monotonic()

        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self._timer = self.create_timer(1.0 / publish_hz, self._on_timer)
        self.get_logger().info(
            "Publishing Gazebo demo commands for gantry XYZ and arm1 joints."
        )

    def _on_timer(self) -> None:
        t = time.monotonic() - self._t0
        wt = 2.0 * math.pi * float(self.get_parameter("frequency_hz").value) * t
        for joint, publisher in self._publishers.items():
            msg = Float64()
            msg.data = self._amp_by_joint[joint] * math.sin(wt + self._phase_by_joint[joint])
            publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = GazeboDemoJointCommands()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
