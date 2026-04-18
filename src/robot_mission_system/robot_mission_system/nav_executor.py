# Copyright 2026 Robot Mission System
"""Nav2 NavigateToPose action client: send, poll, cancel."""

from __future__ import annotations

import logging
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose

_LOG = logging.getLogger(__name__)


class NavExecutor:
    def __init__(
        self,
        node: Node,
        action_name: str = 'navigate_to_pose',
        server_wait_timeout_sec: float = 10.0,
    ) -> None:
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._server_wait_timeout_sec = server_wait_timeout_sec
        self._goal_handle: Optional[ClientGoalHandle] = None
        self._result_future: Any = None

    @property
    def has_active_goal(self) -> bool:
        return self._goal_handle is not None and self._result_future is not None

    def wait_for_server(self) -> bool:
        return self._client.wait_for_server(timeout_sec=self._server_wait_timeout_sec)

    def cancel(self) -> None:
        if self._goal_handle is None:
            return
        try:
            self._goal_handle.cancel_goal_async()
        except Exception as e:  # noqa: BLE001
            _LOG.warning('Cancel goal failed: %s', e)
        self._goal_handle = None
        self._result_future = None

    def send_pose(self, pose: PoseStamped) -> tuple[bool, str]:
        """Begin navigation. Returns (accepted, message)."""
        if not self.wait_for_server():
            return False, 'navigate_to_pose action server not available'
        if self.has_active_goal:
            return False, 'goal already active'

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self._client.send_goal_async(goal)
        self._node.get_logger().info('NavigateToPose sent')
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=5.0)
        gh = send_future.result()
        if gh is None:
            return False, 'send_goal timed out'
        if not gh.accepted:
            return False, 'goal rejected'
        self._goal_handle = gh
        self._result_future = gh.get_result_async()
        return True, 'accepted'

    def poll_result(self) -> Optional[tuple[int, str]]:
        """
        If result ready, returns (status, detail) and clears handles.
        If still running, returns None.
        """
        if self._result_future is None:
            return None
        if not self._result_future.done():
            return None
        try:
            res = self._result_future.result()
            status = res.status
        except Exception as e:  # noqa: BLE001
            self._goal_handle = None
            self._result_future = None
            return -1, f'result_error: {e}'
        self._goal_handle = None
        self._result_future = None
        return status, f'status={status}'
