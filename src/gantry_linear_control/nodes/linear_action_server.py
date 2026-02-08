import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from gantry_linear_control.action import MoveLinear
from gantry_linear_control.drivers.serial_driver import SerialGantryDriver


class LinearActionServer(Node):
    def __init__(self):
        super().__init__("linear_action_server")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("goal_timeout_s", 120.0)
        self.declare_parameter("pos_tolerance_mm", 0.25)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self._driver = SerialGantryDriver(port=port, baud=int(baud))
        self._driver.open()

        self._as = ActionServer(
            self,
            MoveLinear,
            "gantry/move_linear",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("MoveLinear action server ready on /gantry/move_linear")

    def goal_callback(self, goal_request):
        # Basic validation
        if goal_request.feed_mm_s <= 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        try:
            self._driver.stop()
        except Exception:
            pass
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = MoveLinear.Feedback()

        poll_hz = self.get_parameter("poll_hz").get_parameter_value().double_value
        timeout_s = self.get_parameter("goal_timeout_s").get_parameter_value().double_value
        tol = self.get_parameter("pos_tolerance_mm").get_parameter_value().double_value

        # Send move
        try:
            self._driver.move_linear(goal.x_mm, goal.y_mm, goal.z_mm, goal.feed_mm_s)
        except Exception as e:
            goal_handle.abort()
            result = MoveLinear.Result()
            result.success = False
            result.message = f"Failed to send command: {e}"
            return result

        # Wait for acceptance (OK) (optional but helpful)
        t0 = time.time()
        while self._driver.last_ok_age_s() > 0.5:
            if time.time() - t0 > 2.0:
                break
            time.sleep(0.02)

        # Track progress by distance-to-go
        start = self._driver.get_state_copy()
        total_dist = ((goal.x_mm - start.x_mm) ** 2 + (goal.y_mm - start.y_mm) ** 2 + (goal.z_mm - start.z_mm) ** 2) ** 0.5
        total_dist = max(total_dist, 1e-6)

        deadline = time.time() + timeout_s
        period = 1.0 / max(poll_hz, 1e-3)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveLinear.Result()
                result.success = False
                result.message = "Canceled"
                return result

            st = self._driver.get_state_copy()

            # Any error?
            if st.last_error:
                goal_handle.abort()
                result = MoveLinear.Result()
                result.success = False
                result.message = f"MCU error: {st.last_error}"
                return result

            dx = goal.x_mm - st.x_mm
            dy = goal.y_mm - st.y_mm
            dz = goal.z_mm - st.z_mm
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5

            # Feedback
            feedback.progress_0_1 = float(max(0.0, min(1.0, 1.0 - dist / total_dist)))
            feedback.status = "moving" if st.busy else "settling"
            goal_handle.publish_feedback(feedback)

            # Done?
            if dist <= tol and not st.busy:
                goal_handle.succeed()
                result = MoveLinear.Result()
                result.success = True
                result.message = "Arrived"
                return result

            if time.time() > deadline:
                goal_handle.abort()
                result = MoveLinear.Result()
                result.success = False
                result.message = "Timeout waiting for arrival"
                return result

            time.sleep(period)

        goal_handle.abort()
        result = MoveLinear.Result()
        result.success = False
        result.message = "ROS shutting down"
        return result

    def destroy_node(self):
        try:
            self._driver.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LinearActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
