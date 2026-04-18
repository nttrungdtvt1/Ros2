# Copyright 2026 Robot Mission System
"""CLI helper: publish mission commands and teach requests (no custom .srv required)."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_mission_system.waypoint_repository import WaypointRepository


def _pub_string(node: Node, topic: str, data: str) -> None:
    pub = node.create_publisher(String, topic, 10)
    # Allow discovery
    end = time.monotonic() + 2.0
    msg = String(data=data)
    while time.monotonic() < end and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)


def main() -> None:
    parser = argparse.ArgumentParser(description='robot_mission_system operator CLI')
    sub = parser.add_subparsers(dest='action', required=True)

    p_save = sub.add_parser('save', help='Request teach node to save current AMCL pose')
    p_save.add_argument('name', type=str, help='Waypoint name')

    p_start = sub.add_parser('start', help='Start mission with waypoint order')
    p_start.add_argument('names', nargs='+', help='Waypoint names in order')

    sub.add_parser('pause', help='Pause mission')
    sub.add_parser('resume', help='Resume mission')
    sub.add_parser('cancel', help='Cancel mission')
    sub.add_parser('reload', help='Reload waypoint file on mission_manager')

    p_list = sub.add_parser('list', help='List waypoints from storage file')
    p_list.add_argument(
        '--storage',
        type=str,
        default=str(Path.home() / '.robot_mission' / 'waypoints.yaml'),
        help='Path to waypoints.yaml',
    )

    p_pub = sub.add_parser('publish-snapshot-wait', help='Wait one snapshot (debug)')
    p_pub.add_argument('--timeout', type=float, default=3.0)

    args = parser.parse_args()

    if args.action == 'list':
        repo = WaypointRepository(args.storage)
        for n in repo.list_names():
            print(n)
        return

    rclpy.init()
    node = Node('operator_cli')

    try:
        if args.action == 'save':
            _pub_string(node, '/robot_mission/save_waypoint', args.name)
            print(f"Requested save for '{args.name}'")
        elif args.action == 'start':
            payload = json.dumps({'cmd': 'start', 'waypoints': args.names})
            _pub_string(node, '/robot_mission/command', payload)
            print('Mission start published:', payload)
        elif args.action == 'pause':
            _pub_string(node, '/robot_mission/command', json.dumps({'cmd': 'pause'}))
        elif args.action == 'resume':
            _pub_string(node, '/robot_mission/command', json.dumps({'cmd': 'resume'}))
        elif args.action == 'cancel':
            _pub_string(node, '/robot_mission/command', json.dumps({'cmd': 'cancel'}))
        elif args.action == 'reload':
            _pub_string(node, '/robot_mission/command', json.dumps({'cmd': 'reload_repo'}))
        elif args.action == 'publish-snapshot-wait':
            last = {'data': ''}

            def cb(msg: String) -> None:
                last['data'] = msg.data

            node.create_subscription(String, '/robot_mission/snapshot', cb, 10)
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline and not last['data']:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(last['data'] or '(no snapshot)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    sys.exit(0)
