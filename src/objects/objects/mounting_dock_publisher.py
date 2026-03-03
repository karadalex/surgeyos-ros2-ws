#!/usr/bin/env python3

import struct
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class MountingDockPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mounting_dock_publisher")

        self.declare_parameter("topic", "/visualization_marker")
        self.declare_parameter("frame_id", "body")
        self.declare_parameter("marker_ns", "objects")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("mesh_resource", "package://objects/meshes/mounting_dock.stl")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("position.x", 0.2)
        self.declare_parameter("position.y", -1.5)
        self.declare_parameter("position.z", 0.1)
        self.declare_parameter("orientation.x", 0.0)
        self.declare_parameter("orientation.y", 0.0)
        self.declare_parameter("orientation.z", 0.70710678)
        self.declare_parameter("orientation.w", 0.70710678)
        self.declare_parameter("scale.x", 0.001)
        self.declare_parameter("scale.y", 0.001)
        self.declare_parameter("scale.z", 0.001)
        self.declare_parameter("color.r", 0.0)
        self.declare_parameter("color.g", 1.0)
        self.declare_parameter("color.b", 0.0)
        self.declare_parameter("color.a", 1.0)

        mesh_resource = str(self.get_parameter("mesh_resource").value)
        self._mesh_path = self._resolve_mesh_path(mesh_resource)
        self._mesh_points = self._load_stl_triangles(self._mesh_path)
        topic = str(self.get_parameter("topic").value)
        self._pub = self.create_publisher(Marker, topic, 10)
        publish_hz = max(float(self.get_parameter("publish_hz").value), 1e-3)
        self._timer = self.create_timer(1.0 / publish_hz, self._publish_marker)

        self.get_logger().info(
            f"Publishing mounting dock triangles on {topic} from {self._mesh_path}"
        )

    def _resolve_mesh_path(self, mesh_resource: str) -> Path:
        if mesh_resource.startswith("package://objects/"):
            relative_path = mesh_resource.removeprefix("package://objects/")
            share_dir = Path(get_package_share_directory("objects"))
            return share_dir / relative_path
        if mesh_resource.startswith("file://"):
            return Path(mesh_resource.removeprefix("file://"))
        return Path(mesh_resource)

    def _load_stl_triangles(self, mesh_path: Path) -> list[Point]:
        data = mesh_path.read_bytes()
        if self._looks_like_ascii_stl(data):
            return self._load_ascii_stl(data.decode("utf-8", errors="ignore"))
        return self._load_binary_stl(data)

    def _looks_like_ascii_stl(self, data: bytes) -> bool:
        prefix = data[:256].decode("utf-8", errors="ignore").lstrip()
        return prefix.startswith("solid") and b"facet" in data[:1024]

    def _load_ascii_stl(self, text: str) -> list[Point]:
        points: list[Point] = []
        for line in text.splitlines():
            line = line.strip()
            if not line.startswith("vertex "):
                continue
            _, x_str, y_str, z_str = line.split()
            point = Point()
            point.x = float(x_str)
            point.y = float(y_str)
            point.z = float(z_str)
            points.append(point)
        return points

    def _load_binary_stl(self, data: bytes) -> list[Point]:
        if len(data) < 84:
            raise ValueError(f"STL file is too small: {self._mesh_path}")
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        offset = 84
        points: list[Point] = []
        for _ in range(triangle_count):
            if offset + 50 > len(data):
                break
            offset += 12  # skip normal
            for _ in range(3):
                x, y, z = struct.unpack_from("<fff", data, offset)
                point = Point()
                point.x = x
                point.y = y
                point.z = z
                points.append(point)
                offset += 12
            offset += 2  # attribute byte count
        return points

    def _publish_marker(self) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = self._make_base_marker(stamp)
        msg.type = Marker.TRIANGLE_LIST

        msg.scale.x = float(self.get_parameter("scale.x").value)
        msg.scale.y = float(self.get_parameter("scale.y").value)
        msg.scale.z = float(self.get_parameter("scale.z").value)
        msg.color.r = float(self.get_parameter("color.r").value)
        msg.color.g = float(self.get_parameter("color.g").value)
        msg.color.b = float(self.get_parameter("color.b").value)
        msg.color.a = float(self.get_parameter("color.a").value)
        msg.points = self._mesh_points

        self.get_logger().debug(
            "Publishing marker frame=%s mesh=%s triangles=%d pos=(%.3f, %.3f, %.3f) scale=(%.3f, %.3f, %.3f)"
            % (
                msg.header.frame_id,
                self._mesh_path,
                len(self._mesh_points) // 3,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.scale.x,
                msg.scale.y,
                msg.scale.z,
            )
        )
        self._pub.publish(msg)

    def _make_base_marker(self, stamp) -> Marker:
        msg = Marker()
        msg.header.frame_id = str(self.get_parameter("frame_id").value).strip()
        msg.header.stamp = stamp
        msg.ns = str(self.get_parameter("marker_ns").value)
        msg.id = int(self.get_parameter("marker_id").value)
        msg.action = Marker.ADD
        msg.frame_locked = True
        msg.pose.position.x = float(self.get_parameter("position.x").value)
        msg.pose.position.y = float(self.get_parameter("position.y").value)
        msg.pose.position.z = float(self.get_parameter("position.z").value)
        msg.pose.orientation.x = float(self.get_parameter("orientation.x").value)
        msg.pose.orientation.y = float(self.get_parameter("orientation.y").value)
        msg.pose.orientation.z = float(self.get_parameter("orientation.z").value)
        msg.pose.orientation.w = float(self.get_parameter("orientation.w").value)
        return msg


def main() -> None:
    rclpy.init()
    node = MountingDockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
