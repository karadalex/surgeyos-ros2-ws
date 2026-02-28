#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from gantry_linear_control.msg import GantryState as GantryStateMsg
from gantry_linear_control.drivers.serial_driver import SerialGantryDriver


class SerialDriverNode(Node):
    def __init__(self):
        super().__init__("serial_driver_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("state_rate_hz", 20.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        rate = self.get_parameter("state_rate_hz").get_parameter_value().double_value

        self._driver = SerialGantryDriver(
            port=port,
            baud=int(baud),
            line_cb=lambda l: self.get_logger().debug(f"RX: {l}"),
        )
        self._driver.open()
        self.get_logger().info(f"Connected to NodeMCU on {port} @ {baud}")

        self._pub = self.create_publisher(GantryStateMsg, "gantry/state", 10)

        self._home_srv = self.create_service(Trigger, "gantry/home", self._on_home)
        self._stop_srv = self.create_service(Trigger, "gantry/stop", self._on_stop)

        self._timer = self.create_timer(1.0 / max(rate, 1e-3), self._tick)

        # Expose driver to other nodes via composition patterns if desired;
        # For simplicity, action server can open its own driver OR you can run them in same process.

    def _tick(self):
        st = self._driver.get_state_copy()
        msg = GantryStateMsg()
        msg.x_mm = float(st.x_mm)
        msg.y_mm = float(st.y_mm)
        msg.z_mm = float(st.z_mm)
        msg.busy = bool(st.busy)
        msg.last_error = st.last_error
        self._pub.publish(msg)

    def _on_home(self, req, res):
        try:
            self._driver.home()
            res.success = True
            res.message = "HOME sent"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _on_stop(self, req, res):
        try:
            self._driver.stop()
            res.success = True
            res.message = "STOP sent"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def destroy_node(self):
        try:
            self._driver.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
