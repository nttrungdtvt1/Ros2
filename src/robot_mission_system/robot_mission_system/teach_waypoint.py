# Copyright 2026 Robot Mission System
"""Teach mode: save current robot pose as a named waypoint using TF2 (Universal Standard)."""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_mission_system.waypoint_repository import WaypointRecord, WaypointRepository


class TeachWaypointNode(Node):
    def __init__(self) -> None:
        super().__init__('teach_waypoint')
        default_path = str(Path.home() / '.robot_mission' / 'waypoints.yaml')
        
        # Parameters
        self.declare_parameter('storage_path', default_path)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link') # Đồng bộ với cấu hình SLAM/Nav2 của bạn

        raw_path = self.get_parameter('storage_path').get_parameter_value().string_value
        self._path = os.path.expanduser(raw_path)
        self._repo = WaypointRepository(self._path)

        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value

        # TÁI CẤU TRÚC: Khởi tạo TF2 Buffer và Listener thay vì dùng topic AMCL
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(String, '/robot_mission/save_waypoint', self._on_save_name, 10)
        
        self.get_logger().info(
            f'teach_waypoint ready; storage={self._path} | '
            f'Using TF2: ({self._map_frame} -> {self._robot_frame})'
        )

    def _on_save_name(self, msg: String) -> None:
        name = msg.data.strip()
        if not name:
            self.get_logger().warning('Empty waypoint name ignored')
            return

        try:
            # Tra cứu tọa độ thực tế của robot trên bản đồ thông qua cây TF
            now = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0) # Đợi tối đa 1s nếu TF bị trễ
            )
        except TransformException as ex:
            self.get_logger().error(f'Could not get transform {self._map_frame} to {self._robot_frame}: {ex}')
            self.get_logger().error(f"Cannot save '{name}'. Is SLAM or AMCL running?")
            return

        # Trích xuất dữ liệu từ Transform
        t = trans.transform.translation
        r = trans.transform.rotation

        rec = WaypointRecord(
            name=name,
            frame_id=self._map_frame,
            x=t.x,
            y=t.y,
            z=t.z,
            qx=r.x,
            qy=r.y,
            qz=r.z,
            qw=r.w,
            meta={'source': 'tf2_lookup'},
        )
        self._repo.upsert(rec)
        self.get_logger().info(f"Saved waypoint '{name}' -> {self._path} | [X:{t.x:.2f}, Y:{t.y:.2f}]")


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