#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, String


class MuxNode(Node):
    """
    Command multiplexer for aeb_f110.

    Subscribes to throttle/steering from all known sources and forwards only
    the active source's commands to the AEB safety layer. The active source
    can be changed at runtime by publishing to /aeb_f110/mux/active_source.

    A watchdog timer monitors the active source's publish frequency. If no
    message arrives within source_timeout seconds, throttle is forced to 0.0
    until the source recovers.

    Known sources:
        auto    — /aeb_f110/sources/auto/{throttle,steering}
        teleop  — /aeb_f110/sources/teleop/{throttle,steering}
    """

    SOURCES = ('auto', 'teleop')

    def __init__(self):
        super().__init__('mux_node')

        # --- Parameters ---
        self.declare_parameter('active_source', 'auto')
        self.declare_parameter('source_timeout', 0.5)   # [s]

        self._active_source  = self.get_parameter('active_source').value
        self._source_timeout = self.get_parameter('source_timeout').value

        if self._active_source not in self.SOURCES:
            self.get_logger().warning(
                f'[MUX] Unknown active_source "{self._active_source}", '
                f'falling back to "auto"'
            )
            self._active_source = 'auto'

        self.get_logger().info(
            f'Mux node started | active_source={self._active_source} '
            f'| source_timeout={self._source_timeout} s'
        )

        # --- QoS ---
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Source state: latest values + timestamp per source ---
        self._sources: dict = {
            name: {'throttle': 0.0, 'steering': 0.0, 'stamp': None}
            for name in self.SOURCES
        }

        # --- Subscribers — one pair per source ---
        for name in self.SOURCES:
            self.create_subscription(
                Float32,
                f'/aeb_f110/sources/{name}/throttle',
                lambda msg, n=name: self._throttle_cb(msg, n),
                qos,
            )
            self.create_subscription(
                Float32,
                f'/aeb_f110/sources/{name}/steering',
                lambda msg, n=name: self._steering_cb(msg, n),
                qos,
            )

        # Active source selector
        self.create_subscription(
            String, '/aeb_f110/mux/active_source', self._source_select_cb, qos
        )

        # --- Publishers ---
        self._throttle_pub = self.create_publisher(
            Float32, '/aeb_f110/throttle_request', qos
        )
        self._steering_pub = self.create_publisher(
            Float32, '/aeb_f110/steering_request', qos
        )

        # --- Watchdog + forward timer (50 Hz) ---
        self._watchdog_active = False
        self.create_timer(1.0 / 50.0, self._timer_cb)

    # ------------------------------------------------------------------
    # Source callbacks
    # ------------------------------------------------------------------

    def _throttle_cb(self, msg: Float32, source: str) -> None:
        self._sources[source]['throttle'] = msg.data
        self._sources[source]['stamp']    = self.get_clock().now()

    def _steering_cb(self, msg: Float32, source: str) -> None:
        self._sources[source]['steering'] = msg.data
        # stamp no se actualiza aquí — el watchdog solo monitorea throttle,
        # que es el canal de seguridad crítico.

    def _source_select_cb(self, msg: String) -> None:
        requested = msg.data.strip()
        if requested not in self.SOURCES:
            self.get_logger().warning(
                f'[MUX] Requested unknown source "{requested}", ignored. '
                f'Known sources: {self.SOURCES}'
            )
            return
        if requested != self._active_source:
            self.get_logger().info(
                f'[MUX] Source switched: {self._active_source} → {requested}'
            )
            self._active_source  = requested
            self._watchdog_active = False

    # ------------------------------------------------------------------
    # Watchdog + forward timer
    # ------------------------------------------------------------------

    def _timer_cb(self) -> None:
        state    = self._sources[self._active_source]
        now      = self.get_clock().now()
        stamp    = state['stamp']

        if stamp is None:
            # Source has never published — safety: send zero throttle
            self._publish(0.0, 0.0)
            if not self._watchdog_active:
                self.get_logger().warning(
                    f'[MUX] Watchdog: source "{self._active_source}" has not published yet'
                )
                self._watchdog_active = True
            return

        age = (now - stamp).nanoseconds * 1e-9

        if age > self._source_timeout:
            self._publish(0.0, 0.0)
            if not self._watchdog_active:
                self.get_logger().warning(
                    f'[MUX] Watchdog: source "{self._active_source}" timeout '
                    f'({age:.2f} s > {self._source_timeout} s) — throttle zeroed'
                )
                self._watchdog_active = True
        else:
            if self._watchdog_active:
                self.get_logger().info(
                    f'[MUX] Source "{self._active_source}" recovered'
                )
                self._watchdog_active = False
            self._publish(state['throttle'], state['steering'])

    def _publish(self, throttle: float, steering: float) -> None:
        t_msg = Float32()
        t_msg.data = float(throttle)
        self._throttle_pub.publish(t_msg)

        s_msg = Float32()
        s_msg.data = float(steering)
        self._steering_pub.publish(s_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
