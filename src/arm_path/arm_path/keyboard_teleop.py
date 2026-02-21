#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'target')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('step_xy_m', 0.01)
        self.declare_parameter('step_z_m', 0.01)
        self.declare_parameter('start_x_m', 0.18)
        self.declare_parameter('start_y_m', 0.00)
        self.declare_parameter('start_z_m', 0.18)
        self.declare_parameter('pitch_rad', 0.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.step_xy = float(self.get_parameter('step_xy_m').value)
        self.step_z = float(self.get_parameter('step_z_m').value)
        self.x = float(self.get_parameter('start_x_m').value)
        self.y = float(self.get_parameter('start_y_m').value)
        self.z = float(self.get_parameter('start_z_m').value)
        self.pitch = float(self.get_parameter('pitch_rad').value)

        self.pub = self.create_publisher(TFMessage, '/tf', 10)

        rate_hz = max(1.0, float(self.get_parameter('rate_hz').value))
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self._saved_terminal = None
        if sys.stdin.isatty():
            self._saved_terminal = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            self.get_logger().warn('stdin is not a TTY; keyboard input disabled.')

        self.get_logger().info(
            'Keyboard teleop started: arrows=XY, w=+Z, a=-Z, q=quit'
        )
        self._log_pose()

    def _log_pose(self):
        self.get_logger().info(f'target xyz = ({self.x:.3f}, {self.y:.3f}, {self.z:.3f})')

    def _read_key(self):
        if self._saved_terminal is None:
            return None

        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None

        ch = sys.stdin.read(1)
        if ch != '\x1b':
            return ch

        # Arrow keys are ESC + '[' + letter
        ready, _, _ = select.select([sys.stdin], [], [], 0.001)
        if not ready:
            return ch
        seq = sys.stdin.read(2)
        return ch + seq

    def _handle_key(self, key):
        changed = False
        if key == '\x1b[A':  # up
            self.x += self.step_xy
            changed = True
        elif key == '\x1b[B':  # down
            self.x -= self.step_xy
            changed = True
        elif key == '\x1b[D':  # left
            self.y += self.step_xy
            changed = True
        elif key == '\x1b[C':  # right
            self.y -= self.step_xy
            changed = True
        elif key == 'w':
            self.z += self.step_z
            changed = True
        elif key == 'a':
            self.z -= self.step_z
            changed = True
        elif key == 'q':
            self.get_logger().info('Quit requested by keyboard.')
            raise KeyboardInterrupt

        if changed:
            self._log_pose()

    def _publish_target_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        # Pitch-only orientation around Y axis.
        half = 0.5 * self.pitch
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(half)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(half)

        self.pub.publish(TFMessage(transforms=[t]))

    def on_timer(self):
        key = self._read_key()
        if key is not None:
            self._handle_key(key)
        self._publish_target_tf()

    def destroy_node(self):
        if self._saved_terminal is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._saved_terminal)
            self._saved_terminal = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
