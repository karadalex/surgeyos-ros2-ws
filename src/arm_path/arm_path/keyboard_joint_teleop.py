#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class KeyboardJointTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_joint_teleop')

        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('step_rad', 0.05)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
        self.declare_parameter('start_positions', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('min_positions', [-3.14, -3.14, -3.14, -3.14, -3.14])
        self.declare_parameter('max_positions', [3.14, 3.14, 3.14, 3.14, 3.14])

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.step_rad = float(self.get_parameter('step_rad').value)
        self.start_positions = [float(v) for v in self.get_parameter('start_positions').value]
        self.min_positions = [float(v) for v in self.get_parameter('min_positions').value]
        self.max_positions = [float(v) for v in self.get_parameter('max_positions').value]

        self._validate_configuration()

        self.positions = list(self.start_positions)
        self.pub = self.create_publisher(
            JointState,
            str(self.get_parameter('output_topic').value),
            10,
        )

        rate_hz = max(1.0, float(self.get_parameter('rate_hz').value))
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self._saved_terminal = None
        if sys.stdin.isatty():
            self._saved_terminal = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            self.get_logger().warn('stdin is not a TTY; keyboard input disabled.')

        self._keymap = {
            'q': (0, 1.0),
            'a': (0, -1.0),
            'w': (1, 1.0),
            's': (1, -1.0),
            'e': (2, 1.0),
            'd': (2, -1.0),
            'r': (3, 1.0),
            'f': (3, -1.0),
            't': (4, 1.0),
            'g': (4, -1.0),
        }

        self.get_logger().info(
            'Keyboard joint teleop started: q/a=j1, w/s=j2, e/d=j3, r/f=j4, t/g=j5, 0=reset, x=quit'
        )
        self._log_positions()

    def _validate_configuration(self):
        expected_len = 5
        values = (
            ('joint_names', self.joint_names),
            ('start_positions', self.start_positions),
            ('min_positions', self.min_positions),
            ('max_positions', self.max_positions),
        )
        for name, value in values:
            if len(value) != expected_len:
                raise ValueError(f'{name} must contain exactly {expected_len} entries.')

        for idx, (min_pos, start_pos, max_pos) in enumerate(
            zip(self.min_positions, self.start_positions, self.max_positions),
            start=1,
        ):
            if min_pos > max_pos:
                raise ValueError(f'joint {idx}: min_positions must be <= max_positions.')
            if not min_pos <= start_pos <= max_pos:
                raise ValueError(f'joint {idx}: start_positions must be within min/max limits.')

    def _log_positions(self):
        formatted = ', '.join(
            f'{name}={position:.3f}'
            for name, position in zip(self.joint_names, self.positions)
        )
        self.get_logger().info(f'target joints = [{formatted}]')

    def _clamp_positions(self):
        for idx in range(len(self.positions)):
            self.positions[idx] = max(
                self.min_positions[idx],
                min(self.positions[idx], self.max_positions[idx]),
            )

    def _read_key(self):
        if self._saved_terminal is None:
            return None

        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None

        return sys.stdin.read(1)

    def _handle_key(self, key):
        changed = False
        if key in self._keymap:
            joint_idx, direction = self._keymap[key]
            self.positions[joint_idx] += direction * self.step_rad
            changed = True
        elif key == '0':
            self.positions = list(self.start_positions)
            changed = True
        elif key == 'x':
            self.get_logger().info('Quit requested by keyboard.')
            raise KeyboardInterrupt

        if changed:
            self._clamp_positions()
            self._log_positions()

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(self.positions)
        self.pub.publish(msg)

    def on_timer(self):
        key = self._read_key()
        if key is not None:
            self._handle_key(key)
        self._publish_joint_state()

    def destroy_node(self):
        if self._saved_terminal is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._saved_terminal)
            self._saved_terminal = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardJointTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
