#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32


class LinearDriverNode(Node):
    """
    Autonomous linear driver for F1TENTH on AutoDRIVE.

    Publishes a constant forward throttle and zero steering at a fixed rate
    to /aeb_f110/sources/auto/*. The mux_node arbitrates between this source
    and others (e.g. teleop) before forwarding to the AEB safety layer.
    """

    def __init__(self):
        super().__init__('linear_driver_node')

        # --- Parameters ---
        self.declare_parameter('throttle', 0.3)        # Forward throttle [0.0, 1.0]
        self.declare_parameter('steering', 0.0)        # Steering angle (0.0 = straight)
        self.declare_parameter('publish_rate', 20.0)   # Command publish rate [Hz]

        self._throttle     = self.get_parameter('throttle').value
        self._steering     = self.get_parameter('steering').value
        self._publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(
            f'Linear driver node started | throttle={self._throttle} '
            f'| steering={self._steering} '
            f'| rate={self._publish_rate} Hz'
        )

        # --- QoS — matches AEB node profile ---
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Publishers ---
        self._throttle_pub = self.create_publisher(
            Float32, '/aeb_f110/sources/auto/throttle', qos
        )
        self._steering_pub = self.create_publisher(
            Float32, '/aeb_f110/sources/auto/steering', qos
        )

        # --- Timer ---
        self.create_timer(1.0 / self._publish_rate, self._publish_callback)

    def _publish_callback(self) -> None:
        throttle_msg = Float32()
        throttle_msg.data = float(self._throttle)
        self._throttle_pub.publish(throttle_msg)

        steering_msg = Float32()
        steering_msg.data = float(self._steering)
        self._steering_pub.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LinearDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
