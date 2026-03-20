#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

HELP = """
┌──────────────────────────────────────┐
│      AEB F110 — Mode Switcher        │
│                                      │
│  [A]  Switch to AUTO  (linear driver)│
│  [T]  Switch to TELEOP               │
│  [Q]  Quit                           │
└──────────────────────────────────────┘
"""


class ModeSwitcherNode(Node):
    """
    Keyboard-driven source selector for the mux_node.

    Reads single keypresses from stdin (non-blocking via select) and publishes
    the chosen source name to /aeb_f110/mux/active_source.

    Keys:
        a / A  → 'auto'
        t / T  → 'teleop'
        q / Q  → shutdown
    """

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

        # Switch stdin to raw mode (no echo, immediate char delivery)
        tty.setraw(self._fd)

        print(HELP, flush=True)
        print('Current mode: AUTO\n', flush=True)

        # Poll stdin at 10 Hz — non-blocking
        self.create_timer(0.1, self._key_poll)

    def _key_poll(self) -> None:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return

        ch = sys.stdin.read(1).lower()

        if ch == 'q':
            print('\nMode switcher shutting down...', flush=True)
            self._restore_terminal()
            rclpy.shutdown()
            return

        if ch in self.KEY_MAP:
            source = self.KEY_MAP[ch]
            msg = String()
            msg.data = source
            self._pub.publish(msg)
            print(f'\r → Switched to: {source.upper():<8}', flush=True)

    def _restore_terminal(self) -> None:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def destroy_node(self) -> None:
        self._restore_terminal()
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
