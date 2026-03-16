#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Bool, Float32

import numpy as np


class AEBState:
    NORMAL  = 'NORMAL'
    BRAKING = 'BRAKING'


class AEBNode(Node):

    def __init__(self):
        super().__init__('aeb_node')

        # --- Parameters ---
        self.declare_parameter('wheel_radius', 0.058)              # F1TENTH simulated wheel radius [m]
        self.declare_parameter('ttc_threshold', 0.6)               # Minimum TTC before danger flag [s]
        # Effective minimum valid range — set just above the sensor's hardware
        # minimum (0.06 m) to discard noise floor readings without masking
        # real close-range obstacles.
        self.declare_parameter('range_min_cutoff', 0.07)           # Effective minimum valid range [m]
        self.declare_parameter('range_floor', 0.3)                 # Minimum safe range [m]
        self.declare_parameter('range_floor_min_speed', 0.7)       # Min speed to apply range floor [m/s]
        self.declare_parameter('angular_window_deg', 45.0)         # Half-width of TTC sector [deg]
        # The range floor uses a narrower window than the TTC check.
        # Wide-angle beams hitting lateral walls produce short ranges even when
        # the vehicle is not on a collision course; a tighter window eliminates
        # those false positives while still catching close frontal obstacles.
        self.declare_parameter('range_floor_angular_window_deg', 30.0)
        # Throttle applied while the brake latch is active [-1, 0).
        # Negative values command active deceleration; 0.0 means coast only.
        # Active braking is cut off once |speed| drops below brake_stop_speed
        # to prevent the vehicle from reversing unintentionally.
        self.declare_parameter('brake_command', 0.0)               # Braking throttle [-1, 0]
        self.declare_parameter('brake_stop_speed', 0.05)           # Speed threshold to stop active braking [m/s]

        self._wheel_radius               = self.get_parameter('wheel_radius').value
        self._ttc_threshold              = self.get_parameter('ttc_threshold').value
        self._range_min_cutoff           = self.get_parameter('range_min_cutoff').value
        self._range_floor                = self.get_parameter('range_floor').value
        self._range_floor_min_spd        = self.get_parameter('range_floor_min_speed').value
        self._angular_window             = np.deg2rad(self.get_parameter('angular_window_deg').value)
        self._range_floor_angular_window = np.deg2rad(self.get_parameter('range_floor_angular_window_deg').value)
        self._brake_command              = self.get_parameter('brake_command').value
        self._brake_stop_speed           = self.get_parameter('brake_stop_speed').value

        self.get_logger().info(
            f'AEB node started | wheel_radius={self._wheel_radius} m '
            f'| ttc_threshold={self._ttc_threshold} s '
            f'| range_floor={self._range_floor} m '
            f'(min_speed={self._range_floor_min_spd} m/s, '
            f'window=±{self.get_parameter("range_floor_angular_window_deg").value}°) '
            f'| ttc_window=±{self.get_parameter("angular_window_deg").value}° '
            f'| brake_command={self._brake_command}'
        )

        # --- QoS ---
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
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
        # Throttle and steering requests from the operator (teleop remapped here)
        self.create_subscription(
            Float32, '/aeb_f110/throttle_request', self._throttle_request_callback, qos
        )
        self.create_subscription(
            Float32, '/aeb_f110/steering_request', self._steering_request_callback, qos
        )

        # --- Publishers ---
        self._danger_pub       = self.create_publisher(Bool,    '/aeb_f110/danger',                       qos)
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
        angle = msg.position[0]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._left_time_prev is not None:
            dt = t - self._left_time_prev
            if dt > 0.0:
                omega = (angle - self._left_angle_prev) / dt   # [rad/s]
                self._left_speed = omega * self._wheel_radius   # [m/s]

        self._left_angle_prev = angle
        self._left_time_prev = t
        self._speed = (self._left_speed + self._right_speed) / 2.0

    def _right_encoder_callback(self, msg: JointState) -> None:
        angle = msg.position[0]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._right_time_prev is not None:
            dt = t - self._right_time_prev
            if dt > 0.0:
                omega = (angle - self._right_angle_prev) / dt   # [rad/s]
                self._right_speed = omega * self._wheel_radius   # [m/s]

        self._right_angle_prev = angle
        self._right_time_prev = t
        self._speed = (self._left_speed + self._right_speed) / 2.0

    # ------------------------------------------------------------------
    # Operator request callbacks — arbitration point
    # ------------------------------------------------------------------

    def _throttle_request_callback(self, msg: Float32) -> None:
        requested = msg.data

        if self._state == AEBState.BRAKING:
            # Release latch only on explicit reverse request from the operator.
            # This ensures the vehicle resumes only by deliberate input, not by
            # the operator simply holding a forward key that was already set.
            if requested < 0.0:
                self._state = AEBState.NORMAL
                self.get_logger().info('[AEB] Latch released — reverse requested by operator')
                output = requested
            else:
                # Apply braking throttle until nearly stopped, then hold at 0
                # to avoid reversing unintentionally before the latch is released.
                output = self._brake_command if abs(self._speed) > self._brake_stop_speed else 0.0
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

        num_beams = len(ranges)
        angles = msg.angle_min + np.arange(num_beams) * msg.angle_increment

        valid = (
            np.isfinite(ranges)
            & (ranges > self._range_min_cutoff)
            & (ranges < msg.range_max)
        )
        ranges = ranges[valid]
        angles = angles[valid]

        danger_msg = Bool()
        v = self._speed

        if abs(v) < 0.01 or len(ranges) == 0:
            danger_msg.data = False
            self._danger_pub.publish(danger_msg)
            return

        # Angular window filter
        in_window = np.abs(angles) <= self._angular_window
        ranges = ranges[in_window]
        angles = angles[in_window]

        if len(ranges) == 0:
            danger_msg.data = False
            self._danger_pub.publish(danger_msg)
            return

        # iTTC (F1TENTH Lab 2)
        #   range_rate_i = v · cos(θᵢ)
        #   TTC_i = r_i / range_rate_i   (only where range_rate_i > 0)
        range_rates = v * np.cos(angles)
        approaching = range_rates > 0.0

        if not np.any(approaching):
            danger_msg.data = False
            self._danger_pub.publish(danger_msg)
            return

        approaching_ranges = ranges[approaching]
        ttc = approaching_ranges / range_rates[approaching]
        min_ttc   = float(np.min(ttc))
        min_range = float(np.min(approaching_ranges))

        ttc_danger = min_ttc < self._ttc_threshold

        # Range floor uses its own narrower angular window to avoid triggering
        # on lateral walls that fall within the wider TTC sector.
        in_range_window  = np.abs(angles) <= self._range_floor_angular_window
        range_floor_mask = in_range_window & approaching
        if np.any(range_floor_mask):
            range_floor_min = float(np.min(ranges[range_floor_mask]))
        else:
            range_floor_min = float('inf')
        range_danger = (range_floor_min < self._range_floor) and (abs(v) >= self._range_floor_min_spd)

        danger = ttc_danger or range_danger
        danger_msg.data = danger
        self._danger_pub.publish(danger_msg)

        if danger and self._state == AEBState.NORMAL:
            self._state = AEBState.BRAKING
            # Publish brake command immediately without waiting for the next
            # teleop message — at 3.4 Hz lidar vs 10 Hz teleop, reacting
            # here cuts worst-case latency from ~300 ms to ~100 ms.
            throttle_msg = Float32()
            throttle_msg.data = float(self._brake_command)
            self._throttle_cmd_pub.publish(throttle_msg)
            reason = []
            if ttc_danger:
                reason.append(f'TTC={min_ttc:.3f} s < {self._ttc_threshold} s')
            if range_danger:
                reason.append(f'range={min_range:.3f} m < {self._range_floor} m')
            self.get_logger().warn(
                f'[AEB] BRAKING latched | speed={v:+.3f} m/s | {" | ".join(reason)}'
            )
        elif danger and self._state == AEBState.BRAKING:
            self.get_logger().debug(
                f'[AEB] BRAKING active  | speed={v:+.3f} m/s '
                f'| min_TTC={min_ttc:.3f} s | min_range={min_range:.3f} m'
            )
        else:
            self.get_logger().debug(
                f'[AEB] clear   | speed={v:+.3f} m/s | min_TTC={min_ttc:.3f} s '
                f'| min_range={min_range:.3f} m'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
