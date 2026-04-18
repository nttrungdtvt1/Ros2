# Copyright 2026 Robot Mission System
"""Final approach: in-place yaw alignment via cmd_vel_precision (twist_mux priority over Nav)."""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from rclpy.node import Node


def shortest_angular_distance(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PrecisionArrival:
    """Publish angular velocity on cmd_vel_precision until yaw error is small (step-wise API)."""

    def __init__(
        self,
        node: Node,
        cmd_topic: str = 'cmd_vel_precision',
        yaw_tolerance: float = 0.08,
        max_omega: float = 0.35,
        min_omega: float = 0.12,
    ) -> None:
        self._node = node
        self._pub = node.create_publisher(Twist, cmd_topic, 10)
        self._yaw_tolerance = yaw_tolerance
        self._max_omega = max_omega
        self._min_omega = min_omega

    def align_yaw_step(self, current_yaw: float, target_yaw: float) -> tuple[Twist, bool]:
        """One control step; returns (twist, done). Caller runs in timer at ~20 Hz."""
        err = shortest_angular_distance(current_yaw, target_yaw)
        if abs(err) < self._yaw_tolerance:
            return Twist(), True
        cmd = Twist()
        sign = 1.0 if err > 0 else -1.0
        mag = min(self._max_omega, max(self._min_omega, abs(err) * 1.5))
        cmd.angular.z = sign * mag
        return cmd, False

    def publish_zero(self) -> None:
        self._pub.publish(Twist())

    def publish_step(self, current_yaw: float, target_yaw: float) -> bool:
        """Publish one twist step; returns True when aligned."""
        twist, done = self.align_yaw_step(current_yaw, target_yaw)
        self._pub.publish(twist)
        return done
