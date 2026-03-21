from __future__ import annotations

import math
import threading
import time
from typing import Optional

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from gantry_controller.serial_driver import SerialGantryDriver


class GantryControllerNode(Node):
    """Consumes XYZ targets and dispatches them to the gantry MCU."""

    def __init__(self) -> None:
        super().__init__("gantry_controller")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("input_topic", "gantry/target_point")
        self.declare_parameter("position_topic", "gantry/current_point")
        self.declare_parameter("busy_topic", "gantry/busy")
        self.declare_parameter("error_topic", "gantry/last_error")
        self.declare_parameter("state_rate_hz", 20.0)
        self.declare_parameter("default_feed_mm_s", 40.0)
        self.declare_parameter("arrival_tolerance_mm", 0.25)
        self.declare_parameter("command_settle_s", 0.20)
        self.declare_parameter("clamp_to_workspace", False)
        self.declare_parameter("x_min_mm", 0.0)
        self.declare_parameter("x_max_mm", 1500.0)
        self.declare_parameter("y_min_mm", 0.0)
        self.declare_parameter("y_max_mm", 1500.0)
        self.declare_parameter("z_min_mm", 0.0)
        self.declare_parameter("z_max_mm", 300.0)

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        input_topic = str(self.get_parameter("input_topic").value)
        position_topic = str(self.get_parameter("position_topic").value)
        busy_topic = str(self.get_parameter("busy_topic").value)
        error_topic = str(self.get_parameter("error_topic").value)
        state_rate_hz = max(float(self.get_parameter("state_rate_hz").value), 1e-3)

        self._driver = SerialGantryDriver(
            port=port,
            baud=baud,
            line_cb=lambda line: self.get_logger().debug(f"MCU: {line}"),
        )
        self._driver.open()

        self._position_pub = self.create_publisher(Point, position_topic, 10)
        self._busy_pub = self.create_publisher(Bool, busy_topic, 10)
        self._error_pub = self.create_publisher(String, error_topic, 10)
        self._target_sub = self.create_subscription(
            Point,
            input_topic,
            self._on_target,
            10,
        )
        self._home_srv = self.create_service(Trigger, "gantry/home", self._on_home)
        self._stop_srv = self.create_service(Trigger, "gantry/stop", self._on_stop)
        self._timer = self.create_timer(1.0 / state_rate_hz, self._on_timer)

        self._pending_lock = threading.Lock()
        self._pending_target: Optional[tuple[float, float, float]] = None
        self._next_dispatch_at = 0.0
        self._last_published_error = ""

        self.get_logger().info(f"Connected to gantry MCU on {port} @ {baud}")
        self.get_logger().info(f"Listening for Point targets on {input_topic}")

    def _on_target(self, msg: Point) -> None:
        target = self._normalize_target(msg)
        if target is None:
            return

        with self._pending_lock:
            self._pending_target = target

        self._dispatch_pending_if_idle()

    def _on_timer(self) -> None:
        self._dispatch_pending_if_idle()
        state = self._driver.get_state_copy()

        point_msg = Point()
        point_msg.x = state.x_mm
        point_msg.y = state.y_mm
        point_msg.z = state.z_mm
        self._position_pub.publish(point_msg)

        busy_msg = Bool()
        busy_msg.data = state.busy
        self._busy_pub.publish(busy_msg)

        error_msg = String()
        error_msg.data = state.last_error
        self._error_pub.publish(error_msg)

        if state.last_error and state.last_error != self._last_published_error:
            self.get_logger().error(f"MCU error: {state.last_error}")
        self._last_published_error = state.last_error

    def _dispatch_pending_if_idle(self) -> None:
        now = time.monotonic()
        if now < self._next_dispatch_at:
            return

        state = self._driver.get_state_copy()
        if state.busy:
            return

        with self._pending_lock:
            target = self._pending_target
            self._pending_target = None

        if target is None:
            return

        tolerance = float(self.get_parameter("arrival_tolerance_mm").value)
        if self._within_tolerance(target, state, tolerance):
            return

        feed_mm_s = float(self.get_parameter("default_feed_mm_s").value)
        try:
            self._driver.move_linear(*target, feed_mm_s=feed_mm_s)
        except Exception as exc:
            self.get_logger().error(f"Failed to send gantry move: {exc}")
            with self._pending_lock:
                if self._pending_target is None:
                    self._pending_target = target
            return

        self._next_dispatch_at = now + float(
            self.get_parameter("command_settle_s").value
        )
        self.get_logger().info(
            "Dispatched gantry target "
            f"x={target[0]:.2f}mm "
            f"y={target[1]:.2f}mm "
            f"z={target[2]:.2f}mm"
        )

    def _normalize_target(self, msg: Point) -> Optional[tuple[float, float, float]]:
        values = (float(msg.x), float(msg.y), float(msg.z))
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error("Rejected target with non-finite coordinates")
            return None

        bounds = (
            (
                "x",
                values[0],
                float(self.get_parameter("x_min_mm").value),
                float(self.get_parameter("x_max_mm").value),
            ),
            (
                "y",
                values[1],
                float(self.get_parameter("y_min_mm").value),
                float(self.get_parameter("y_max_mm").value),
            ),
            (
                "z",
                values[2],
                float(self.get_parameter("z_min_mm").value),
                float(self.get_parameter("z_max_mm").value),
            ),
        )
        clamp = bool(self.get_parameter("clamp_to_workspace").value)

        normalized = []
        for axis_name, value, axis_min, axis_max in bounds:
            if axis_min <= value <= axis_max:
                normalized.append(value)
                continue
            if clamp:
                clamped = min(max(value, axis_min), axis_max)
                self.get_logger().warning(
                    f"Clamped {axis_name} from {value:.2f}mm to {clamped:.2f}mm"
                )
                normalized.append(clamped)
                continue
            self.get_logger().error(
                "Rejected target: "
                f"{axis_name}={value:.2f}mm "
                f"outside [{axis_min:.2f}, {axis_max:.2f}]"
            )
            return None

        return tuple(normalized)

    @staticmethod
    def _within_tolerance(
        target: tuple[float, float, float],
        state,
        tolerance_mm: float,
    ) -> bool:
        return (
            abs(target[0] - state.x_mm) <= tolerance_mm
            and abs(target[1] - state.y_mm) <= tolerance_mm
            and abs(target[2] - state.z_mm) <= tolerance_mm
        )

    def _on_home(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            self._driver.home()
            with self._pending_lock:
                self._pending_target = None
            self._next_dispatch_at = time.monotonic() + float(
                self.get_parameter("command_settle_s").value
            )
            response.success = True
            response.message = "HOME sent to gantry MCU"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_stop(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            self._driver.stop()
            with self._pending_lock:
                self._pending_target = None
            self._next_dispatch_at = time.monotonic() + float(
                self.get_parameter("command_settle_s").value
            )
            response.success = True
            response.message = "STOP sent to gantry MCU"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def destroy_node(self) -> bool:
        try:
            self._driver.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GantryControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
