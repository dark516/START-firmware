#!/usr/bin/env python3
import sys
import select
import tty
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


KEY_BINDINGS = {
    'w': (0.1, 0.0),   # forward
    's': (-0.1, 0.0),  # backward
    'a': (0.0,  0.5), # rotate right (negative z)
    'd': (0.0,  -0.5),  # rotate left (positive z)
    ' ': (0.0, 0.0),   # space = stop
}


class WasdTeleop(Node):
    def __init__(self) -> None:
        super().__init__('wasd_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('🎮 WASD teleop started. w/s forward/back, a/d rotate, space stop, q quit')

    def _get_key(self) -> str:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return key

    def _publish(self, lin_x: float, ang_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.publisher.publish(msg)
        self.get_logger().info(f"/cmd_vel: {lin_x:.2f} {ang_z:.2f}")

    def run(self) -> None:
        try:
            while rclpy.ok():
                key = self._get_key().lower()
                if not key:
                    continue
                if key in ('q', '\u0003'):
                    break
                if key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[key]
                    self._publish(lin, ang)
        except KeyboardInterrupt:
            pass
        finally:
            self._publish(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WasdTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


