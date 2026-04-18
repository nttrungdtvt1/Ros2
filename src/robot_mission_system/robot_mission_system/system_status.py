# Copyright 2026 Robot Mission System
"""Lightweight status aggregator: logs /robot_mission/snapshot at configurable rate."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SystemStatusNode(Node):
    def __init__(self) -> None:
        super().__init__('system_status')
        self.declare_parameter('snapshot_topic', '/robot_mission/snapshot')
        self.declare_parameter('log_each_message', False)
        self._last: str = ''
        topic = self.get_parameter('snapshot_topic').get_parameter_value().string_value
        self.create_subscription(String, topic, self._on_snap, 10)
        self.get_logger().info('system_status listening on %s', topic)

    def _on_snap(self, msg: String) -> None:
        if self.get_parameter('log_each_message').get_parameter_value().bool_value:
            try:
                data = json.loads(msg.data)
                self.get_logger().info('snapshot %s', json.dumps(data, sort_keys=True))
            except json.JSONDecodeError:
                self.get_logger().warning('bad snapshot json')
        self._last = msg.data

    def get_last_snapshot_json(self) -> str:
        return self._last


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SystemStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
