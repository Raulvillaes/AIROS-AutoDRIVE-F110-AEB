#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

INFO = """
---------------------------------------
AEB F110 - Mode Switcher
---------------------------------------

A : Switch to AUTO  (linear driver)
T : Switch to TELEOP

Press CTRL+C to quit

NOTE: Press keys within this terminal
---------------------------------------
"""


class ModeSwitcherNode(Node):
    KEY_MAP = {
        'a': 'auto',
        't': 'teleop',
    }

    def __init__(self):
        super().__init__('mode_switcher_node')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self.create_publisher(String, '/aeb_f110/mux/active_source', qos)

        # Save terminal settings so we can restore them on exit
        self._fd           = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)

        # Switch stdin to cbreak mode (no echo, immediate char delivery, SIGINT preserved)
        tty.setcbreak(self._fd)

        try:
            print(INFO, flush=True)
            print('Current mode: AUTO', flush=True)

            # Poll stdin at 10 Hz — non-blocking
            self.create_timer(0.1, self._key_poll)
        except Exception:
            self._restore_terminal()
            raise

    def _key_poll(self) -> None:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return

        ch = sys.stdin.read(1).lower()

        if ch in self.KEY_MAP:
            source = self.KEY_MAP[ch]
            msg = String()
            msg.data = source
            self._pub.publish(msg)
            print(f'\r → Switched to: {source.upper():<8}', flush=True)

    def _restore_terminal(self) -> None:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def destroy_node(self) -> None:
        try:
            self._restore_terminal()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
