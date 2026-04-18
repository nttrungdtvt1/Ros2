# Copyright 2026 Robot Mission System
"""Teach mode: save current AMCL pose as a named waypoint."""

from __future__ import annotations

import os
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_mission_system.waypoint_repository import WaypointRecord, WaypointRepository


class TeachWaypointNode(Node):
    def __init__(self) -> None:
        super().__init__('teach_waypoint')
        default_path = str(Path.home() / '.robot_mission' / 'waypoints.yaml')
        self.declare_parameter('storage_path', default_path)
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        raw_path = self.get_parameter('storage_path').get_parameter_value().string_value
        self._path = os.path.expanduser(raw_path)
        self._repo = WaypointRepository(self._path)
        self._last_amcl: PoseWithCovarianceStamped | None = None
        amcl_topic = self.get_parameter('amcl_pose_topic').get_parameter_value().string_value
        self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self._on_amcl,
            10,
        )
        self.create_subscription(String, '/robot_mission/save_waypoint', self._on_save_name, 10)
        self.get_logger().info(
            'teach_waypoint ready; storage=%s subscribe=%s',
            self._path,
            amcl_topic,
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_amcl = msg

    def _on_save_name(self, msg: String) -> None:
        name = msg.data.strip()
        if not name:
            self.get_logger().warning('Empty waypoint name ignored')
            return
        if self._last_amcl is None:
            self.get_logger().error('No AMCL pose received yet; cannot save %s', name)
            return
        p = self._last_amcl.pose.pose.position
        o = self._last_amcl.pose.pose.orientation
        rec = WaypointRecord(
            name=name,
            frame_id=self._last_amcl.header.frame_id or 'map',
            x=p.x,
            y=p.y,
            z=p.z,
            qx=o.x,
            qy=o.y,
            qz=o.z,
            qw=o.w,
            meta={'source': 'amcl_pose'},
        )
        self._repo.upsert(rec)
        self.get_logger().info("Saved waypoint '%s' -> %s", name, self._path)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TeachWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
