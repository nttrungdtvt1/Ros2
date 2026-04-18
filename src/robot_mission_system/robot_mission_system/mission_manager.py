# Copyright 2026 Robot Mission System
"""Mission queue: Nav2 goals, pause/resume/cancel, precision yaw, obstacle gating."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from robot_mission_system.nav_executor import NavExecutor
from robot_mission_system.obstacle_supervisor import ObstacleSupervisor
from robot_mission_system.precision_arrival import PrecisionArrival, quaternion_to_yaw
from robot_mission_system.waypoint_repository import WaypointRepository


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager')
        default_path = str(Path.home() / '.robot_mission' / 'waypoints.yaml')
        self.declare_parameter('storage_path', default_path)
        self.declare_parameter('command_topic', '/robot_mission/command')
        self.declare_parameter('snapshot_topic', '/robot_mission/snapshot')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('repeat_route', False)
        self.declare_parameter('enable_precision_yaw', True)
        self.declare_parameter('max_consecutive_failures', 3)
        self.declare_parameter('require_scan_before_goal', True)
        self.declare_parameter('precision_phase_timeout_sec', 20.0)

        storage_raw = self.get_parameter('storage_path').get_parameter_value().string_value
        self._repo = WaypointRepository(os.path.expanduser(storage_raw))
        self._nav = NavExecutor(
            self,
            action_name=self.get_parameter('navigate_action').get_parameter_value().string_value,
        )
        self._obstacle = ObstacleSupervisor(self)
        self._precision = PrecisionArrival(self)
        self._command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.create_subscription(String, self._command_topic, self._on_command, 10)

        self._snapshot_pub = self.create_publisher(
            String,
            self.get_parameter('snapshot_topic').get_parameter_value().string_value,
            10,
        )
        self._last_amcl_yaw: Optional[float] = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

        self._state = 'IDLE'  # IDLE | RUNNING | PAUSED | FAILED | PRECISION
        self._queue: List[str] = []
        self._idx = 0
        self._fail_streak = 0
        self._paused_from_running = False
        self._repeat = self.get_parameter('repeat_route').get_parameter_value().bool_value

        self._goal_timeout = self.get_parameter('goal_timeout_sec').get_parameter_value().double_value
        self._goal_deadline: Optional[float] = None
        self._use_precision = self.get_parameter('enable_precision_yaw').get_parameter_value().bool_value
        self._require_scan = self.get_parameter('require_scan_before_goal').get_parameter_value().bool_value
        self._max_fails = self.get_parameter('max_consecutive_failures').get_parameter_value().integer_value
        self._precision_timeout = self.get_parameter(
            'precision_phase_timeout_sec'
        ).get_parameter_value().double_value

        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._precision_target_yaw: Optional[float] = None
        self._precision_deadline: Optional[float] = None
        self._waypoint_send_pending = False

        self.create_timer(0.05, self._tick_nav)
        self.create_timer(0.2, self._publish_snapshot)
        self.create_timer(1.0 / 20.0, self._tick_precision)

        self.get_logger().info('mission_manager command topic %s', self._command_topic)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        o = msg.pose.pose.orientation
        self._last_amcl_yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)

    def _on_command(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Invalid JSON command: %s', msg.data)
            return
        cmd = str(data.get('cmd', '')).lower()
        self.get_logger().info('command %s', data)

        if cmd == 'start':
            names = data.get('waypoints') or []
            if not isinstance(names, list) or not names:
                self.get_logger().warning('start requires waypoints array')
                return
            self._repo.reload()
            self._queue = [str(x) for x in names]
            self._idx = 0
            self._fail_streak = 0
            self._state = 'RUNNING'
            self._paused_from_running = False
            self._start_current_waypoint()
        elif cmd == 'pause':
            if self._state in ('RUNNING', 'PRECISION'):
                self._paused_from_running = True
                self._state = 'PAUSED'
                self._nav.cancel()
                self._precision.publish_zero()
                self._goal_deadline = None
                self._precision_deadline = None
        elif cmd == 'resume':
            if self._state == 'PAUSED' and self._paused_from_running:
                self._paused_from_running = False
                self._state = 'RUNNING'
                self._start_current_waypoint()
        elif cmd == 'cancel':
            self._nav.cancel()
            self._precision.publish_zero()
            self._state = 'IDLE'
            self._queue = []
            self._idx = 0
            self._goal_deadline = None
            self._precision_deadline = None
            self._waypoint_send_pending = False
        elif cmd == 'reload_repo':
            self._repo.reload()
        else:
            self.get_logger().warning('unknown cmd %s', cmd)

    def _start_current_waypoint(self) -> None:
        if self._state == 'PAUSED':
            return
        if self._idx >= len(self._queue):
            if self._repeat and self._queue:
                self._idx = 0
            else:
                self.get_logger().info('Mission complete')
                self._state = 'IDLE'
                self._queue = []
                self._waypoint_send_pending = False
                return

        if self._require_scan and not self._obstacle.scan_ok():
            self.get_logger().warning('Waiting: scan not healthy')
            self._waypoint_send_pending = True
            return

        self._waypoint_send_pending = False
        name = self._queue[self._idx]
        rec = self._repo.get(name)
        if rec is None:
            self.get_logger().error("Unknown waypoint '%s'", name)
            self._state = 'FAILED'
            self._waypoint_send_pending = False
            return

        pose = PoseStamped()
        pose.header.frame_id = rec.frame_id or self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = rec.x
        pose.pose.position.y = rec.y
        pose.pose.position.z = rec.z
        pose.pose.orientation.x = rec.qx
        pose.pose.orientation.y = rec.qy
        pose.pose.orientation.z = rec.qz
        pose.pose.orientation.w = rec.qw

        ok, detail = self._nav.send_pose(pose)
        if not ok:
            self.get_logger().error('send_pose failed: %s', detail)
            self._fail_streak += 1
            if self._fail_streak >= self._max_fails:
                self._state = 'FAILED'
            return
        self._goal_deadline = self.get_clock().now().nanoseconds / 1e9 + self._goal_timeout
        self.get_logger().info("Navigating to '%s' (%d/%d)", name, self._idx + 1, len(self._queue))

    def _tick_nav(self) -> None:
        if self._state == 'RUNNING' and self._waypoint_send_pending and not self._nav.has_active_goal:
            if not self._require_scan or self._obstacle.scan_ok():
                self._waypoint_send_pending = False
                self._start_current_waypoint()
        if self._state != 'RUNNING':
            return
        if not self._nav.has_active_goal:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self._goal_deadline is not None and now > self._goal_deadline:
            self.get_logger().warning('Goal timeout; cancel and retry same leg')
            self._nav.cancel()
            self._fail_streak += 1
            if self._fail_streak >= self._max_fails:
                self._state = 'FAILED'
            else:
                self._start_current_waypoint()
            return

        res = self._nav.poll_result()
        if res is None:
            return
        status, detail = res
        self.get_logger().info('Nav result %s', detail)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._fail_streak = 0
            if self._use_precision:
                rec = self._repo.get(self._queue[self._idx])
                if rec is not None:
                    self._precision_target_yaw = quaternion_to_yaw(rec.qx, rec.qy, rec.qz, rec.qw)
                    self._state = 'PRECISION'
                    self._precision_deadline = now + self._precision_timeout
                else:
                    self._advance_queue()
            else:
                self._advance_queue()
        else:
            self._fail_streak += 1
            self.get_logger().warning('Nav did not succeed (status=%s)', status)
            if self._fail_streak >= self._max_fails:
                self._state = 'FAILED'
            else:
                self._start_current_waypoint()

    def _tick_precision(self) -> None:
        if self._state != 'PRECISION':
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self._precision_deadline is not None and now > self._precision_deadline:
            self.get_logger().warning('Precision phase timeout')
            self._precision.publish_zero()
            self._advance_queue()
            return
        if self._last_amcl_yaw is None or self._precision_target_yaw is None:
            return
        done = self._precision.publish_step(self._last_amcl_yaw, self._precision_target_yaw)
        if done:
            self._precision.publish_zero()
            self._advance_queue()

    def _advance_queue(self) -> None:
        self._state = 'RUNNING'
        self._idx += 1
        self._precision_target_yaw = None
        self._goal_deadline = None
        self._precision_deadline = None
        self._start_current_waypoint()

    def _publish_snapshot(self) -> None:
        snap = {
            'state': self._state,
            'queue': list(self._queue),
            'index': self._idx,
            'fail_streak': self._fail_streak,
            'obstacle': self._obstacle.snapshot(),
        }
        msg = String()
        msg.data = json.dumps(snap)
        self._snapshot_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exe.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
