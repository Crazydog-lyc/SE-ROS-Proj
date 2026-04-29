import sys
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
            return SimpleNamespace(sec=0, nanosec=0)

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


def test_wait_until_ready():
    navigator = FakeNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        localizer="smoother_server",
        task_result_constants=FakeTaskResult,
    )

    assert adapter.wait_until_ready(1.0) is True
    assert navigator.wait_called == "smoother_server"


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


def test_cancel_task_calls_underlying_navigator():
    navigator = FakeNavigator()
    adapter = BasicNavigatorAdapter(
        navigator=navigator,
        task_result_constants=FakeTaskResult,
    )

    adapter.send_goal(WaypointSpec("P1", 1.0, 2.0, 0.0))
    adapter.cancel_task()

    assert navigator.cancel_count == 1
