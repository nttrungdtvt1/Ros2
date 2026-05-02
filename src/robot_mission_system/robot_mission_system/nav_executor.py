# # Copyright 2026 Robot Mission System
# """Nav2 NavigateToPose action client: send, poll, cancel."""

# from __future__ import annotations

# import logging
# from typing import Any, Optional

# import rclpy
# from geometry_msgs.msg import PoseStamped
# from rclpy.action import ActionClient
# from rclpy.action.client import ClientGoalHandle
# from rclpy.node import Node

# from nav2_msgs.action import NavigateToPose

# _LOG = logging.getLogger(__name__)


# class NavExecutor:
#     def __init__(
#         self,
#         node: Node,
#         action_name: str = 'navigate_to_pose',
#         server_wait_timeout_sec: float = 10.0,
#     ) -> None:
#         self._node = node
#         self._client = ActionClient(node, NavigateToPose, action_name)
#         self._server_wait_timeout_sec = server_wait_timeout_sec
#         self._goal_handle: Optional[ClientGoalHandle] = None
#         self._result_future: Any = None

#     @property
#     def has_active_goal(self) -> bool:
#         return self._goal_handle is not None and self._result_future is not None

#     def wait_for_server(self) -> bool:
#         return self._client.wait_for_server(timeout_sec=self._server_wait_timeout_sec)

#     def cancel(self) -> None:
#         if self._goal_handle is None:
#             return
#         try:
#             self._goal_handle.cancel_goal_async()
#         except Exception as e:  # noqa: BLE001
#             _LOG.warning('Cancel goal failed: %s', e)
#         self._goal_handle = None
#         self._result_future = None

#     def send_pose(self, pose: PoseStamped) -> tuple[bool, str]:
#         """Begin navigation. Returns (accepted, message)."""
#         if not self.wait_for_server():
#             return False, 'navigate_to_pose action server not available'
#         if self.has_active_goal:
#             return False, 'goal already active'

#         goal = NavigateToPose.Goal()
#         goal.pose = pose
#         send_future = self._client.send_goal_async(goal)
#         self._node.get_logger().info('NavigateToPose sent')
#         rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=5.0)
#         gh = send_future.result()
#         if gh is None:
#             return False, 'send_goal timed out'
#         if not gh.accepted:
#             return False, 'goal rejected'
#         self._goal_handle = gh
#         self._result_future = gh.get_result_async()
#         return True, 'accepted'

#     def poll_result(self) -> Optional[tuple[int, str]]:
#         """
#         If result ready, returns (status, detail) and clears handles.
#         If still running, returns None.
#         """
#         if self._result_future is None:
#             return None
#         if not self._result_future.done():
#             return None
#         try:
#             res = self._result_future.result()
#             status = res.status
#         except Exception as e:  # noqa: BLE001
#             self._goal_handle = None
#             self._result_future = None
#             return -1, f'result_error: {e}'
#         self._goal_handle = None
#         self._result_future = None
#         return status, f'status={status}'



# Copyright 2026 Robot Mission System
"""Nav2 NavigateToPose action client: send, poll, cancel (Fully Asynchronous)."""

from __future__ import annotations

import logging
from typing import Any, Optional

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
        
        # Biến trạng thái để quản lý tiến trình bất đồng bộ (tránh Deadlock)
        self._is_sending = False
        self._terminal_result: Optional[tuple[int, str]] = None

    @property
    def has_active_goal(self) -> bool:
        """Hệ thống đang bận nếu đang trong quá trình gửi lệnh HOẶC đã có goal_handle."""
        return self._is_sending or (self._goal_handle is not None)

    def wait_for_server(self) -> bool:
        return self._client.wait_for_server(timeout_sec=self._server_wait_timeout_sec)

    def cancel(self) -> None:
        """Hủy an toàn không block hệ thống."""
        if self._goal_handle is None:
            return
        try:
            # Gửi yêu cầu hủy bất đồng bộ, hệ thống sẽ trả về trạng thái CANCELED thông qua callback
            self._goal_handle.cancel_goal_async()
            self._node.get_logger().info('Sent cancellation request to Nav2')
        except Exception as e:  # noqa: BLE001
            _LOG.warning('Cancel goal failed: %s', e)

    def send_pose(self, pose: PoseStamped) -> tuple[bool, str]:
        """Bắt đầu điều hướng (HOÀN TOÀN BẤT ĐỒNG BỘ). Trả về (accepted, message)."""
        if not self.wait_for_server():
            return False, 'navigate_to_pose action server not available'
        if self.has_active_goal:
            return False, 'goal already active'

        goal = NavigateToPose.Goal()
        goal.pose = pose
        
        self._is_sending = True
        self._terminal_result = None
        
        # Gửi Goal Async và gắn callback để không chặn Timer của Mission Manager
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_callback)
        
        self._node.get_logger().info('NavigateToPose request sent asynchronously')
        return True, 'accepted'

    def _goal_response_callback(self, future: Any) -> None:
        """Xử lý phản hồi chấp nhận/từ chối từ Nav2 Server."""
        try:
            gh = future.result()
            if gh is None or not gh.accepted:
                self._node.get_logger().warning('Goal rejected by Nav2 server')
                self._terminal_result = (-1, 'goal rejected')
                self._is_sending = False
                return
            
            # Goal được chấp nhận, đăng ký lắng nghe kết quả cuối cùng
            self._goal_handle = gh
            self._is_sending = False
            
            result_future = gh.get_result_async()
            result_future.add_done_callback(self._get_result_callback)
            
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f'Error in goal response: {e}')
            self._terminal_result = (-1, f'send_goal error: {e}')
            self._is_sending = False

    def _get_result_callback(self, future: Any) -> None:
        """Nhận kết quả cuối cùng (Succeeded, Canceled, Aborted)."""
        try:
            res = future.result()
            status = res.status
            self._terminal_result = (status, f'status={status}')
        except Exception as e:  # noqa: BLE001
            self._terminal_result = (-1, f'result_error: {e}')
        finally:
            # Dọn dẹp tay cầm (handle) để hệ thống sẵn sàng nhận waypoint mới
            self._goal_handle = None

    def poll_result(self) -> Optional[tuple[int, str]]:
        """
        Mission Manager gọi hàm này ở mỗi tick. 
        Nếu có kết quả, trả về (status, detail) và dọn dẹp. Nếu đang chạy, trả về None.
        """
        if self._terminal_result is not None:
            res = self._terminal_result
            self._terminal_result = None
            return res
        
        return None