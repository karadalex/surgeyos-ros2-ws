#!/usr/bin/env python3

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.action import ActionClient
from rclpy.node import Node

from gantry_linear_control.action import MoveLinear
from gantry_linear_control.msg import GantryState


@dataclass
class XYZPoint:
    x_mm: float
    y_mm: float
    z_mm: float


class XYZPathPlanner(Node):
    """
    Plans a straight-line Cartesian path in XYZ (mm) and executes it as
    short linear segments through /gantry/move_linear.
    """

    def __init__(self):
        super().__init__("xyz_path_planner")

        self.declare_parameter("segment_length_mm", 25.0)
        self.declare_parameter("default_feed_mm_s", 80.0)
        self.declare_parameter("wait_for_server_s", 5.0)

        # Default bounds mapped from robot_description1 prismatic limits:
        # lower=-1, upper=1 [m] -> [-1000, 1000] [mm].
        self.declare_parameter("x_min_mm", -1000.0)
        self.declare_parameter("x_max_mm", 1000.0)
        self.declare_parameter("y_min_mm", -1000.0)
        self.declare_parameter("y_max_mm", 1000.0)
        self.declare_parameter("z_min_mm", -1000.0)
        self.declare_parameter("z_max_mm", 1000.0)

        # Fallback start position if gantry/state has not arrived yet.
        self.declare_parameter("start_x_mm", 0.0)
        self.declare_parameter("start_y_mm", 0.0)
        self.declare_parameter("start_z_mm", 0.0)

        self._last_state: Optional[GantryState] = None
        self._exec_lock = threading.Lock()
        self._is_executing = False

        self._state_sub = self.create_subscription(
            GantryState, "gantry/state", self._on_state, 10
        )
        self._goal_sub = self.create_subscription(
            Point, "gantry/goal_xyz", self._on_goal_xyz, 10
        )
        self._path_pub = self.create_publisher(PoseArray, "gantry/planned_path", 10)

        self._ac = ActionClient(self, MoveLinear, "gantry/move_linear")
        self.get_logger().info(
            "XYZ planner ready. Send Point(mm) on /gantry/goal_xyz to plan+execute."
        )

    def _on_state(self, msg: GantryState) -> None:
        self._last_state = msg

    def _on_goal_xyz(self, msg: Point) -> None:
        target = XYZPoint(float(msg.x), float(msg.y), float(msg.z))

        with self._exec_lock:
            if self._is_executing:
                self.get_logger().warn("Ignoring new goal: planner is already executing.")
                return
            self._is_executing = True

        th = threading.Thread(target=self._plan_and_execute, args=(target,), daemon=True)
        th.start()

    def _current_point(self) -> XYZPoint:
        if self._last_state is not None:
            return XYZPoint(
                x_mm=float(self._last_state.x_mm),
                y_mm=float(self._last_state.y_mm),
                z_mm=float(self._last_state.z_mm),
            )

        return XYZPoint(
            x_mm=float(self.get_parameter("start_x_mm").value),
            y_mm=float(self.get_parameter("start_y_mm").value),
            z_mm=float(self.get_parameter("start_z_mm").value),
        )

    def _within_bounds(self, p: XYZPoint) -> bool:
        x_min = float(self.get_parameter("x_min_mm").value)
        x_max = float(self.get_parameter("x_max_mm").value)
        y_min = float(self.get_parameter("y_min_mm").value)
        y_max = float(self.get_parameter("y_max_mm").value)
        z_min = float(self.get_parameter("z_min_mm").value)
        z_max = float(self.get_parameter("z_max_mm").value)
        return (
            x_min <= p.x_mm <= x_max
            and y_min <= p.y_mm <= y_max
            and z_min <= p.z_mm <= z_max
        )

    @staticmethod
    def _distance(a: XYZPoint, b: XYZPoint) -> float:
        return math.sqrt(
            (b.x_mm - a.x_mm) ** 2 + (b.y_mm - a.y_mm) ** 2 + (b.z_mm - a.z_mm) ** 2
        )

    def _plan_linear_path(self, start: XYZPoint, goal: XYZPoint) -> List[XYZPoint]:
        segment_length_mm = max(float(self.get_parameter("segment_length_mm").value), 1e-6)
        dist = self._distance(start, goal)
        segments = max(1, int(math.ceil(dist / segment_length_mm)))

        pts: List[XYZPoint] = []
        for i in range(1, segments + 1):
            t = float(i) / float(segments)
            pts.append(
                XYZPoint(
                    x_mm=start.x_mm + t * (goal.x_mm - start.x_mm),
                    y_mm=start.y_mm + t * (goal.y_mm - start.y_mm),
                    z_mm=start.z_mm + t * (goal.z_mm - start.z_mm),
                )
            )
        return pts

    def _publish_path(self, frame_id: str, points: List[XYZPoint]) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        for p in points:
            pose = Pose()
            pose.position.x = p.x_mm / 1000.0
            pose.position.y = p.y_mm / 1000.0
            pose.position.z = p.z_mm / 1000.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def _plan_and_execute(self, target: XYZPoint) -> None:
        try:
            start = self._current_point()

            if not self._within_bounds(target):
                self.get_logger().error(
                    f"Target out of bounds: ({target.x_mm:.1f}, {target.y_mm:.1f}, {target.z_mm:.1f}) mm"
                )
                return
            if not self._within_bounds(start):
                self.get_logger().error(
                    f"Current position out of bounds: ({start.x_mm:.1f}, {start.y_mm:.1f}, {start.z_mm:.1f}) mm"
                )
                return

            waypoints = self._plan_linear_path(start, target)
            self._publish_path("world", waypoints)

            wait_s = float(self.get_parameter("wait_for_server_s").value)
            if not self._ac.wait_for_server(timeout_sec=wait_s):
                self.get_logger().error("MoveLinear action server unavailable.")
                return

            feed = float(self.get_parameter("default_feed_mm_s").value)
            for i, wp in enumerate(waypoints, start=1):
                goal = MoveLinear.Goal()
                goal.x_mm = float(wp.x_mm)
                goal.y_mm = float(wp.y_mm)
                goal.z_mm = float(wp.z_mm)
                goal.feed_mm_s = float(feed)

                send_future = self._ac.send_goal_async(goal)
                while rclpy.ok() and not send_future.done():
                    time.sleep(0.01)
                if not send_future.done():
                    self.get_logger().error("Interrupted while sending waypoint goal.")
                    return

                goal_handle = send_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    self.get_logger().error(f"Waypoint {i}/{len(waypoints)} rejected.")
                    return

                result_future = goal_handle.get_result_async()
                while rclpy.ok() and not result_future.done():
                    time.sleep(0.01)
                if not result_future.done():
                    self.get_logger().error("Interrupted while waiting waypoint result.")
                    return

                result = result_future.result().result
                if not result.success:
                    self.get_logger().error(
                        f"Waypoint {i}/{len(waypoints)} failed: {result.message}"
                    )
                    return

            self.get_logger().info(
                f"Completed path from ({start.x_mm:.1f}, {start.y_mm:.1f}, {start.z_mm:.1f}) "
                f"to ({target.x_mm:.1f}, {target.y_mm:.1f}, {target.z_mm:.1f}) mm."
            )
        finally:
            with self._exec_lock:
                self._is_executing = False


def main():
    rclpy.init()
    node = XYZPathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
