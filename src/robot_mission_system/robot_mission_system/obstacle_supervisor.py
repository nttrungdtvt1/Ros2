# Copyright 2026 Robot Mission System
"""Sensor / Nav health signals for mission decisions (stale scan, close range)."""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional

from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleSupervisor:
    """
    Tracks LaserScan freshness and simple front clearance.
    Does not replace Nav2 local planner; provides operator / mission gating signals.
    """

    def __init__(
        self,
        node: Node,
        scan_topic: str = 'scan',
        max_scan_age_sec: float = 1.0,
        front_arc_deg: float = 60.0,
        min_clearance_m: float = 0.25,
    ) -> None:
        self._node = node
        self._max_scan_age_sec = max_scan_age_sec
        self._front_arc_rad = math.radians(front_arc_deg)
        self._min_clearance_m = min_clearance_m
        self._last_scan: Optional[LaserScan] = None
        self._last_scan_mono: float = 0.0
        self._sub = node.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._last_scan_mono = time.monotonic()

    def scan_age_sec(self) -> Optional[float]:
        if self._last_scan is None:
            return None
        return time.monotonic() - self._last_scan_mono

    def scan_ok(self) -> bool:
        age = self.scan_age_sec()
        if age is None:
            return False
        return age <= self._max_scan_age_sec

    def front_clearance_violation(self) -> bool:
        """True if any ray in forward cone is closer than min_clearance (invalid if no scan)."""
        if self._last_scan is None:
            return True
        msg = self._last_scan
        if not msg.ranges:
            return True
        bad = False
        for i, r in enumerate(msg.ranges):
            an = msg.angle_min + float(i) * msg.angle_increment
            an = math.atan2(math.sin(an), math.cos(an))
            if abs(an) > self._front_arc_rad:
                continue
            if not math.isfinite(r) or r < msg.range_min:
                continue
            if r < self._min_clearance_m + 1e-6:
                bad = True
                break
        return bad

    def snapshot(self) -> Dict[str, Any]:
        age = self.scan_age_sec()
        return {
            'scan_ok': self.scan_ok(),
            'scan_age_sec': age,
            'front_blocked': self.front_clearance_violation(),
        }
