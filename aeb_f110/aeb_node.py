#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float32

import numpy as np


class AEBState:
    NORMAL  = 'NORMAL'
    BRAKING = 'BRAKING'


class AEBNode(Node):

    def __init__(self):
        super().__init__('aeb_node')

        # --- Parameters ---
        self.declare_parameter('wheel_radius', 0.058)           # F1TENTH simulated wheel radius [m]
        self.declare_parameter('ttc_threshold', 0.66)           # Minimum TTC before danger flag [s]
        # Effective minimum valid range — set to the minimun possible distance
        # between the LiDAR and a front wall
        self.declare_parameter('range_min_cutoff', 0.13)        # Effective minimum valid range [m]
        self.declare_parameter('angular_window_deg', 12.5)      # Half-width of TTC sector [deg]
        # Throttle applied while the brake latch is active (0).
        self.declare_parameter('brake_command', 0.0)            # Braking throttle
        self.declare_parameter('min_speed', 0.8)               # Minimum speed to engage AEB [m/s]
        self.declare_parameter('encoder_timeout', 0.5)         # Max age of encoder data before speed is considered stale [s]
        self.declare_parameter('max_wheel_speed', 8.0)         # Physical speed ceiling for spike / wrap-around rejection [m/s]

        self._wheel_radius      = self.get_parameter('wheel_radius').value
        self._ttc_threshold     = self.get_parameter('ttc_threshold').value
        self._range_min_cutoff  = self.get_parameter('range_min_cutoff').value
        self._angular_window    = np.deg2rad(self.get_parameter('angular_window_deg').value)
        self._brake_command     = self.get_parameter('brake_command').value
        self._min_speed         = self.get_parameter('min_speed').value
        self._encoder_timeout   = self.get_parameter('encoder_timeout').value
        self._max_wheel_speed   = self.get_parameter('max_wheel_speed').value

        self.get_logger().info(
            f'AEB node started | wheel_radius={self._wheel_radius} m '
            f'| ttc_threshold={self._ttc_threshold} s '
            f'| ttc_window=±{self.get_parameter("angular_window_deg").value}° '
            f'| brake_command={self._brake_command} '
            f'| min_speed={self._min_speed} m/s '
            f'| encoder_timeout={self._encoder_timeout} s '
            f'| max_wheel_speed={self._max_wheel_speed} m/s'
        )

        # --- QoS ---
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscribers ---
        self.create_subscription(
            LaserScan, '/autodrive/f1tenth_1/lidar', self._lidar_callback, qos
        )
        self.create_subscription(
            JointState, '/autodrive/f1tenth_1/left_encoder', self._left_encoder_callback, qos
        )
        self.create_subscription(
            JointState, '/autodrive/f1tenth_1/right_encoder', self._right_encoder_callback, qos
        )
        # Throttle and steering requests forwarded by the mux node
        self.create_subscription(
            Float32, '/aeb_f110/throttle_request', self._throttle_request_callback, qos
        )
        self.create_subscription(
            Float32, '/aeb_f110/steering_request', self._steering_request_callback, qos
        )

        # --- Publishers ---
        self._throttle_cmd_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command',  qos)
        self._steering_cmd_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command',  qos)

        # --- Encoder state (per wheel) ---
        self._left_angle_prev: float | None = None
        self._left_time_prev: float | None = None
        self._right_angle_prev: float | None = None
        self._right_time_prev: float | None = None

        # Per-wheel linear speed derived directly from encoder differentiation [m/s]
        self._left_speed: float = 0.0
        self._right_speed: float = 0.0

        # Longitudinal speed estimate [m/s]
        self._speed: float = 0.0

        # Timestamp of the last encoder message received (ROS clock)
        self._last_encoder_stamp = None

        # --- AEB state machine ---
        # NORMAL:  pass operator throttle through to the bridge unchanged.
        # BRAKING: override throttle regardless of operator input.
        #          Latch is released only when the operator explicitly requests
        #          reverse (throttle_request < 0), indicating intentional action.
        self._state: str = AEBState.NORMAL

    # ------------------------------------------------------------------
    # Encoder callbacks — derive wheel speed by differentiating position
    # ------------------------------------------------------------------

    def _left_encoder_callback(self, msg: JointState) -> None:
        if not msg.position:
            return
        angle = msg.position[0]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._left_time_prev is not None:
            dt = t - self._left_time_prev
            if dt > 0.0:
                omega = (angle - self._left_angle_prev) / dt   # [rad/s]
                speed = omega * self._wheel_radius              # [m/s]
                # Reject spikes caused by encoder wrap-around or simulator resets.
                if abs(speed) <= self._max_wheel_speed:
                    self._left_speed = speed
                else:
                    self.get_logger().warning(
                        f'[AEB] Left encoder spike discarded: {speed:.2f} m/s'
                    )

        self._left_angle_prev = angle
        self._left_time_prev = t
        self._speed = (self._left_speed + self._right_speed) / 2.0
        # Mark the encoder as alive so the staleness check in _lidar_callback
        # knows this data is fresh.
        self._last_encoder_stamp = self.get_clock().now()

    def _right_encoder_callback(self, msg: JointState) -> None:
        if not msg.position:
            return
        angle = msg.position[0]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._right_time_prev is not None:
            dt = t - self._right_time_prev
            if dt > 0.0:
                omega = (angle - self._right_angle_prev) / dt   # [rad/s]
                speed = omega * self._wheel_radius              # [m/s]
                # Reject spikes caused by encoder wrap-around or simulator resets.
                if abs(speed) <= self._max_wheel_speed:
                    self._right_speed = speed
                else:
                    self.get_logger().warning(
                        f'[AEB] Right encoder spike discarded: {speed:.2f} m/s'
                    )

        self._right_angle_prev = angle
        self._right_time_prev = t
        self._speed = (self._left_speed + self._right_speed) / 2.0
        # Mark the encoder as alive so the staleness check in _lidar_callback
        # knows this data is fresh.
        self._last_encoder_stamp = self.get_clock().now()

    # ------------------------------------------------------------------
    # Operator request callbacks — arbitration point
    # ------------------------------------------------------------------

    def _throttle_request_callback(self, msg: Float32) -> None:
        requested = msg.data

        if self._state == AEBState.BRAKING:
            # Release latch only on explicit reverse request from the operator.
            # This ensures the vehicle resumes only by deliberate input.
            if requested < 0.0:
                self._state = AEBState.NORMAL
                self.get_logger().info('[AEB] Latch released — reverse requested by operator')
                output = requested
            else:
                # Apply and hold throttle at 0
                output = self._brake_command
        else:
            output = requested

        throttle_msg = Float32()
        throttle_msg.data = float(output)
        self._throttle_cmd_pub.publish(throttle_msg)

    def _steering_request_callback(self, msg: Float32) -> None:
        # Steering is always passed through — the operator can steer freely
        # even while the AEB brake latch is active.
        self._steering_cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # LiDAR callback — iTTC computation and state transitions
    # ------------------------------------------------------------------

    def _lidar_callback(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges, dtype=np.float64)

        # Guard: if encoders have never published, speed is unknown — skip iTTC.
        if self._last_encoder_stamp is None:
            return

        # Guard: if the last encoder message is older than encoder_timeout, the
        # speed estimate is stale and cannot be trusted. Skipping iTTC is the
        # safe choice — it prevents the AEB from acting on a ghost speed value
        # that no longer reflects the vehicle's actual state.
        encoder_age = (self.get_clock().now() - self._last_encoder_stamp).nanoseconds * 1e-9
        if encoder_age > self._encoder_timeout:
            self.get_logger().warning(
                f'[AEB] Encoder data stale ({encoder_age:.2f} s) — skipping iTTC'
            )
            return

        v = self._speed

        if abs(v) < self._min_speed:
            return

        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        valid = (
            np.isfinite(ranges)
            & (ranges > self._range_min_cutoff)
            & (ranges < msg.range_max)
        )
        ranges = ranges[valid]
        angles = angles[valid]
        if len(ranges) == 0:
            return

        # Angular window filter
        in_window = np.abs(angles) <= self._angular_window
        ranges = ranges[in_window]
        angles = angles[in_window]

        if len(ranges) == 0:
            return

        # iTTC (F1TENTH Lab 2)
        #   range_rate_i = v · cos(θᵢ)
        #   TTC_i = r_i / range_rate_i   (only where range_rate_i > 0)
        range_rates = v * np.cos(angles)
        approaching = range_rates > 0.0

        if not np.any(approaching):
            return

        approaching_ranges = ranges[approaching]
        ttc = approaching_ranges / range_rates[approaching]
        min_ttc = float(np.min(ttc))

        danger = min_ttc < self._ttc_threshold

        if danger and self._state == AEBState.NORMAL:
            self._state = AEBState.BRAKING
            throttle_msg = Float32()
            throttle_msg.data = float(self._brake_command)
            self._throttle_cmd_pub.publish(throttle_msg)
            self.get_logger().warning(
                f'[AEB] BRAKING latched | speed={v:+.3f} m/s | TTC={min_ttc:.3f} s < {self._ttc_threshold} s'
            )
        elif danger and self._state == AEBState.BRAKING:
            self.get_logger().debug(
                f'[AEB] BRAKING active  | speed={v:+.3f} m/s | min_TTC={min_ttc:.3f} s'
            )
        else:
            self.get_logger().debug(
                f'[AEB] clear   | speed={v:+.3f} m/s | min_TTC={min_ttc:.3f} s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
