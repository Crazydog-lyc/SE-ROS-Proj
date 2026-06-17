# ========================================================================
# 文件: src/nav2_mission_manager/test/test_navigator_adapter.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# 基于 nav2_simple_commander.BasicNavigator 写一个 BasicNavigatorAdapter
# 薄封装：wait_until_ready(timeout)、send_goal(WaypointSpec)、is_task_complete、cancel_task、get_feedback/get_result；waitUntilNav2Active
# 用 daemon thread + Event 做超时，方法体可以先 stub。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
import sys
import time

import pytest
from types import ModuleType, SimpleNamespace

geometry_msgs = ModuleType("geometry_msgs")
geometry_msgs_msg = ModuleType("geometry_msgs.msg")



class FakePoseStamped:
    def __init__(self):
        self.header = SimpleNamespace(frame_id="", stamp=None)
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )


geometry_msgs_msg.PoseStamped = FakePoseStamped
geometry_msgs.msg = geometry_msgs_msg
sys.modules.setdefault("geometry_msgs", geometry_msgs)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

from nav2_mission_manager.models import NavTaskResult, WaypointSpec
from nav2_mission_manager.navigator_adapter import BasicNavigatorAdapter


class FakeTaskResult:
    SUCCEEDED = "SUCCEEDED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"


class FakeClock:
    class _Now:
        @staticmethod
        def to_msg():
            from builtin_interfaces.msg import Time

            return Time(sec=0, nanosec=0)

    @staticmethod
    def now():
        return FakeClock._Now()


class FakeNavigator:
    def __init__(self):
        self.goal = None
        self.wait_called = False
        self.complete = False
        self.result = FakeTaskResult.SUCCEEDED
        self.feedback = None
        self.cancel_count = 0

    def waitUntilNav2Active(self, localizer):
        self.wait_called = localizer

    def get_clock(self):
        return FakeClock()

    def goToPose(self, pose):
        self.goal = pose

    def isTaskComplete(self):
        return self.complete

    def getFeedback(self):
        return self.feedback

    def cancelTask(self):
        self.cancel_count += 1

    def getResult(self):
        return self.result


# adapter mock Nav2
# 单测：def wait_until_ready
def test_wait_until_ready():
    navigator = FakeNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        localizer="smoother_server",
        task_result_constants=FakeTaskResult,
    )

    assert adapter.wait_until_ready(1.0) is True
    assert navigator.wait_called == "smoother_server"


# adapter mock Nav2
# 单测：def send_goal_and_get_result
def test_send_goal_and_get_result():
    navigator = FakeNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        task_result_constants=FakeTaskResult,
    )

    sent = adapter.send_goal(WaypointSpec("P1", 1.0, 2.0, 0.0))
    navigator.complete = True

    assert sent is True
    assert adapter.is_task_complete() is True
    assert adapter.get_result() == NavTaskResult.SUCCEEDED


# adapter mock Nav2
# 单测：def cancel_task_calls_underlying_navigator
def test_cancel_task_calls_underlying_navigator():
    navigator = FakeNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        task_result_constants=FakeTaskResult,
    )

    adapter.send_goal(WaypointSpec("P1", 1.0, 2.0, 0.0))
    adapter.cancel_task()

    assert navigator.cancel_count == 1


def test_send_goal_rejected_on_exception():
    class ExplodingNavigator(FakeNavigator):
        def goToPose(self, pose):
            raise RuntimeError("boom")

    navigator = ExplodingNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        task_result_constants=FakeTaskResult,
    )
    assert adapter.send_goal(WaypointSpec("P1", 0.0, 0.0, 0.0)) is False
    assert adapter.task_active is False


def test_get_feedback_when_inactive():
    navigator = FakeNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        task_result_constants=FakeTaskResult,
    )
    assert adapter.get_feedback() is None


def test_get_feedback_with_duration_fields():
    navigator = FakeNavigator()
    navigator.feedback = SimpleNamespace(
        current_waypoint=1,
        distance_remaining=3.0,
        navigation_time=SimpleNamespace(sec=2, nanosec=500_000_000),
        estimated_time_remaining=SimpleNamespace(sec=1, nanosec=0),
    )
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        task_result_constants=FakeTaskResult,
    )
    adapter.send_goal(WaypointSpec("P1", 0.0, 0.0, 0.0))
    snap = adapter.get_feedback()
    assert snap is not None
    assert snap.navigation_time_sec == pytest.approx(2.5)


def test_get_result_mappings():
    navigator = FakeNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        task_result_constants=FakeTaskResult,
    )
    navigator.result = FakeTaskResult.CANCELED
    assert adapter.get_result() == NavTaskResult.CANCELED
    navigator.result = FakeTaskResult.FAILED
    assert adapter.get_result() == NavTaskResult.FAILED
    navigator.result = "OTHER"
    assert adapter.get_result() == NavTaskResult.UNKNOWN


def test_wait_until_ready_times_out():
    class SlowNavigator(FakeNavigator):
        def waitUntilNav2Active(self, localizer):
            time.sleep(0.2)

    adapter = BasicNavigatorAdapter(
        navigator=SlowNavigator(),
        task_result_constants=FakeTaskResult,
    )
    assert adapter.wait_until_ready(0.01) is False


def test_wait_until_ready_lifecycle_path(monkeypatch):
    class LifecycleNavigator(FakeNavigator):
        def __init__(self):
            super().__init__()
            self.calls = 0

        def create_client(self, srv_type, service_name):
            assert service_name.endswith("/get_state")
            return self

        def wait_for_service(self, timeout_sec=1.0):
            return True

        def call_async(self, request):
            self.calls += 1
            return SimpleNamespace(
                result=lambda: SimpleNamespace(current_state=SimpleNamespace(label="active")),
                done=lambda: True,
            )

    def fake_spin(_node, future, timeout_sec=None):
        return None

    monkeypatch.setattr(
        "nav2_mission_manager.navigator_adapter.rclpy.spin_until_future_complete",
        fake_spin,
    )

    nav = LifecycleNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=nav,
        localizer="smoother_server",
        task_result_constants=FakeTaskResult,
    )
    assert adapter.wait_until_ready(1.0) is True
    assert nav.calls >= 2


def test_get_feedback_none_from_navigator():
    class NoFeedbackNavigator(FakeNavigator):
        def getFeedback(self):
            return None

    adapter = BasicNavigatorAdapter(
        navigator=NoFeedbackNavigator(),
        task_result_constants=FakeTaskResult,
    )
    adapter.send_goal(WaypointSpec("P1", 0.0, 0.0, 0.0))
    assert adapter.get_feedback() is None


def test_is_task_complete_when_already_inactive():
    adapter = BasicNavigatorAdapter(
        navigator=FakeNavigator(),
        task_result_constants=FakeTaskResult,
    )
    assert adapter.is_task_complete() is True


def test_duration_to_seconds_invalid():
    from nav2_mission_manager.navigator_adapter import _duration_to_seconds

    assert _duration_to_seconds(None) is None
    assert _duration_to_seconds(SimpleNamespace(sec=1)) is None


def test_adapter_uses_injected_navigator_cls():
    class StubNavigator:
        def __init__(self):
            self.created = True

    adapter = BasicNavigatorAdapter(
        navigator_cls=StubNavigator,
        task_result_constants=FakeTaskResult,
    )
    assert adapter._navigator.created is True


def test_wait_until_ready_lifecycle_timeout(monkeypatch):
    class PendingLifecycleNavigator(FakeNavigator):
        def create_client(self, srv_type, service_name):
            return self

        def wait_for_service(self, timeout_sec=1.0):
            return False

    clock = {"t": 0.0}

    def fake_monotonic():
        clock["t"] += 0.4
        return clock["t"]

    monkeypatch.setattr(
        "nav2_mission_manager.navigator_adapter.time.monotonic",
        fake_monotonic,
    )
    adapter = BasicNavigatorAdapter(
        navigator=PendingLifecycleNavigator(),
        task_result_constants=FakeTaskResult,
    )
    assert adapter.wait_until_ready(0.5) is False


def test_wait_until_ready_lifecycle_waits_for_active(monkeypatch):
    class DelayedLifecycleNavigator(FakeNavigator):
        def __init__(self):
            super().__init__()
            self.calls = 0

        def create_client(self, srv_type, service_name):
            return self

        def wait_for_service(self, timeout_sec=1.0):
            return True

        def call_async(self, request):
            self.calls += 1
            label = "inactive" if self.calls == 1 else "active"
            return SimpleNamespace(
                result=lambda: SimpleNamespace(
                    current_state=SimpleNamespace(label=label)
                )
            )

    monkeypatch.setattr(
        "nav2_mission_manager.navigator_adapter.rclpy.spin_until_future_complete",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        "nav2_mission_manager.navigator_adapter.time.sleep",
        lambda *_args, **_kwargs: None,
    )

    nav = DelayedLifecycleNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=nav,
        localizer="smoother_server",
        task_result_constants=FakeTaskResult,
    )
    assert adapter.wait_until_ready(2.0) is True


def test_wait_until_ready_amcl_initial_pose(monkeypatch):
    class AmclNavigator(FakeNavigator):
        def __init__(self):
            super().__init__()
            self.calls = 0
            self.initial_pose_received = False
            self.pose_set = False

        def create_client(self, srv_type, service_name):
            return self

        def wait_for_service(self, timeout_sec=1.0):
            return True

        def call_async(self, request):
            self.calls += 1
            return SimpleNamespace(
                result=lambda: SimpleNamespace(
                    current_state=SimpleNamespace(label="active")
                )
            )

        def _setInitialPose(self):
            self.pose_set = True
            self.initial_pose_received = True

    def fake_spin_once(_node, timeout_sec=None):
        return None

    monkeypatch.setattr(
        "nav2_mission_manager.navigator_adapter.rclpy.spin_until_future_complete",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        "nav2_mission_manager.navigator_adapter.rclpy.spin_once",
        fake_spin_once,
    )

    nav = AmclNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=nav,
        localizer="amcl",
        task_result_constants=FakeTaskResult,
    )
    assert adapter.wait_until_ready(2.0) is True
    assert nav.pose_set is True


def test_wait_until_ready_mock_raises():
    class ExplodingNavigator(FakeNavigator):
        def waitUntilNav2Active(self, localizer):
            raise RuntimeError("nav2 unavailable")

    adapter = BasicNavigatorAdapter(
        navigator=ExplodingNavigator(),
        task_result_constants=FakeTaskResult,
    )
    with pytest.raises(RuntimeError, match="nav2 unavailable"):
        adapter.wait_until_ready(1.0)
