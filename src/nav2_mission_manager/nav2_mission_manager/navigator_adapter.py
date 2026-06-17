# ========================================================================
# 文件: src/nav2_mission_manager/nav2_mission_manager/navigator_adapter.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# 基于 nav2_simple_commander.BasicNavigator 写一个 BasicNavigatorAdapter
# 薄封装：wait_until_ready(timeout)、send_goal(WaypointSpec)、is_task_complete、cancel_task、get_feedback/get_result；
# wait_until_ready 在 execute 同线程轮询 lifecycle get_state（mock 仍用带超时的 legacy 线程）。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
import math
import threading
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState

from .models import MissionFeedbackSnapshot, NavTaskResult, WaypointSpec


def _duration_to_seconds(duration_msg: Any) -> Optional[float]:
    if duration_msg is None:
        return None
    sec = getattr(duration_msg, "sec", None)
    nanosec = getattr(duration_msg, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return float(sec) + float(nanosec) / 1_000_000_000.0



class BasicNavigatorAdapter:
    """封装 BasicNavigator，把 WaypointSpec 转成 goToPose 调用。"""

    def __init__(
        self,
        navigator: Any = None,
        navigator_cls: Any = None,
        localizer: str = "smoother_server",
        task_result_constants: Any = None,
    ) -> None:
        if navigator is not None:
            self._navigator = navigator
        else:
            if navigator_cls is None:
                from nav2_simple_commander.robot_navigator import BasicNavigator

                navigator_cls = BasicNavigator
            self._navigator = navigator_cls()
        self._localizer = localizer
        self._task_active = False
        self._lock = threading.Lock()
        if task_result_constants is not None:
            self._task_result_constants = task_result_constants
        else:
            try:
                from nav2_simple_commander.robot_navigator import TaskResult

                self._task_result_constants = TaskResult
            except ImportError:
                self._task_result_constants = None

    @property
    def task_active(self) -> bool:
        with self._lock:
            return self._task_active

    def _wait_lifecycle_node_active(self, node_name: str, deadline: float) -> bool:
        nav = self._navigator
        node_service = f"{node_name}/get_state"
        state_client = nav.create_client(GetState, node_service)
        req = GetState.Request()

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            if not state_client.wait_for_service(timeout_sec=min(1.0, remaining)):
                continue
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(nav, future, timeout_sec=min(1.0, remaining))
            if future.result() is not None and future.result().current_state.label == "active":
                return True
            time.sleep(min(0.2, remaining))
        return False

    def _wait_until_ready_mock(self, timeout_sec: float) -> bool:
        """仅用于无 create_client 的 mock navigator；带超时但不跨线程调 rclpy。"""
        nav = self._navigator
        done = threading.Event()
        error: list[BaseException] = []

        def worker() -> None:
            try:
                nav.waitUntilNav2Active(localizer=self._localizer)
            except BaseException as exc:  # pragma: no cover - defensive
                error.append(exc)
            finally:
                done.set()

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        finished = done.wait(timeout=timeout_sec)
        if error:
            raise error[0]
        return finished

    def wait_until_ready(self, timeout_sec: float) -> bool:
        nav = self._navigator
        deadline = time.monotonic() + timeout_sec

        if hasattr(nav, "create_client"):
            if not self._wait_lifecycle_node_active(self._localizer, deadline):
                return False
            if self._localizer == "amcl" and hasattr(nav, "initial_pose_received"):
                while time.monotonic() < deadline:
                    if nav.initial_pose_received:
                        break
                    if hasattr(nav, "_setInitialPose"):
                        nav._setInitialPose()
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return False
                    rclpy.spin_once(nav, timeout_sec=min(1.0, remaining))
                else:
                    return False
            return self._wait_lifecycle_node_active("bt_navigator", deadline)

        return self._wait_until_ready_mock(timeout_sec)

    def send_goal(self, waypoint: WaypointSpec, frame_id: str = "map") -> bool:
        # yaw 转四元数，和 mission JSON 里的弧度制一致
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self._navigator.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)

        try:
            self._navigator.goToPose(pose)
        except Exception:
            with self._lock:
                self._task_active = False
            return False
        with self._lock:
            self._task_active = True
        return True

    def is_task_complete(self) -> bool:
        with self._lock:
            if not self._task_active:
                return True
        complete = bool(self._navigator.isTaskComplete())
        if complete:
            with self._lock:
                self._task_active = False
        return complete

    def get_feedback(self) -> Optional[MissionFeedbackSnapshot]:
        with self._lock:
            active = self._task_active
        if not active:
            return None
        feedback = self._navigator.getFeedback()
        if feedback is None:
            return None

        return MissionFeedbackSnapshot(
            current_waypoint_index=int(getattr(feedback, "current_waypoint", 0)),
            distance_remaining=getattr(feedback, "distance_remaining", None),
            navigation_time_sec=_duration_to_seconds(
                getattr(feedback, "navigation_time", None)
            ),
            estimated_time_remaining_sec=_duration_to_seconds(
                getattr(feedback, "estimated_time_remaining", None)
            ),
        )

    def cancel_task(self) -> None:
        with self._lock:
            active = self._task_active
        if active:
            self._navigator.cancelTask()

    def get_result(self) -> NavTaskResult:
        result = self._navigator.getResult()
        constants = self._task_result_constants
        if constants is not None:
            if result == constants.SUCCEEDED:
                return NavTaskResult.SUCCEEDED
            if result == constants.CANCELED:
                return NavTaskResult.CANCELED
            if result == constants.FAILED:
                return NavTaskResult.FAILED
        return NavTaskResult.UNKNOWN
