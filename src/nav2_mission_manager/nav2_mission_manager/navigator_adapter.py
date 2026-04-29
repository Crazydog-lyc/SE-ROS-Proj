import math
import threading
from typing import Any, Optional

from geometry_msgs.msg import PoseStamped

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
    """Thin wrapper around nav2_simple_commander.BasicNavigator."""

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
        return self._task_active

    def wait_until_ready(self, timeout_sec: float) -> bool:
        done = threading.Event()
        error: list[BaseException] = []

        def worker() -> None:
            try:
                self._navigator.waitUntilNav2Active(localizer=self._localizer)
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

    def send_goal(self, waypoint: WaypointSpec, frame_id: str = "map") -> bool:
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
            self._task_active = False
            return False
        self._task_active = True
        return True

    def is_task_complete(self) -> bool:
        if not self._task_active:
            return True
        complete = bool(self._navigator.isTaskComplete())
        if complete:
            self._task_active = False
        return complete

    def get_feedback(self) -> Optional[MissionFeedbackSnapshot]:
        if not self._task_active:
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
        if self._task_active:
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
