#!/usr/bin/env python3

import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GantryJointTrajectory(Node):
    """Publish a short joint trajectory for gantry XYZ via joint_trajectory_controller."""

    def __init__(self) -> None:
        super().__init__("gantry_joint_trajectory")

        self.declare_parameter("controller_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter(
            "joint_names",
            [
                "extra_root_2_to_gantry_column",  # forward axis
                "extra_root_1_to_axis1",          # right axis
                "z",                              # vertical axis
            ],
        )
        self.declare_parameter("forward_m", 1.0)
        self.declare_parameter("right_m", 0.5)
        self.declare_parameter("lower_m", 0.5)
        self.declare_parameter("duration_s", 3.0)

        controller_topic = str(self.get_parameter("controller_topic").value)
        self._pub = self.create_publisher(JointTrajectory, controller_topic, 10)
        self._sent = False

        self._timer = self.create_timer(0.2, self._try_publish)
        self.get_logger().info(f"Waiting to publish trajectory on {controller_topic}")

    def _try_publish(self) -> None:
        if self._sent:
            return

        if self._pub.get_subscription_count() == 0:
            self.get_logger().info("No joint_trajectory_controller subscriber yet; waiting...")
            return

        joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        forward_m = float(self.get_parameter("forward_m").value)
        right_m = float(self.get_parameter("right_m").value)
        lower_m = float(self.get_parameter("lower_m").value)
        duration_s = max(0.1, float(self.get_parameter("duration_s").value))

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = joint_names

        start = JointTrajectoryPoint()
        start.positions = [0.0, 0.0, 0.0]
        start.time_from_start = Duration(sec=0, nanosec=0)

        goal = JointTrajectoryPoint()
        goal.positions = [forward_m, right_m, -abs(lower_m)]
        whole_seconds = int(math.floor(duration_s))
        nanos = int((duration_s - whole_seconds) * 1e9)
        goal.time_from_start = Duration(sec=whole_seconds, nanosec=nanos)

        traj.points = [start, goal]
        self._pub.publish(traj)
        self._sent = True
        self.get_logger().info(
            "Published trajectory: forward=%.3f m, right=%.3f m, down=%.3f m in %.2f s"
            % (forward_m, right_m, lower_m, duration_s)
        )


def main() -> None:
    rclpy.init()
    node = GantryJointTrajectory()
    try:
        while rclpy.ok() and not node._sent:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
