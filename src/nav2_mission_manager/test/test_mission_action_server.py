"""MissionActionServer 单元测试（mock navigator，不启 Nav2）。"""
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from builtin_interfaces.msg import Time
from course_interfaces.msg import SafetyState

from nav2_mission_manager.events import Event, EventType
from nav2_mission_manager.mission_action_server import MissionActionServerNode
from nav2_mission_manager.models import (
    MissionExecutionContext,
    MissionFeedbackSnapshot,
    MissionSpec,
    NavTaskResult,
    WaypointSpec,
)
from nav2_mission_manager.navigator_adapter import BasicNavigatorAdapter
from nav2_mission_manager.state_machine import MissionStateMachine
from nav2_mission_manager.states import MissionState


class FakeTaskResult:
    SUCCEEDED = "SUCCEEDED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"


class FakeClock:
    @staticmethod
    def now():
        class _Now:
            @staticmethod
            def to_msg():
                return Time(sec=0, nanosec=0)

        return _Now()


class FakeNavigator:
    def __init__(self):
        self.wait_ready = True
        self.send_ok = True
        self.complete = False
        self.result = FakeTaskResult.SUCCEEDED
        self.task_active = False
        self.feedback = SimpleNamespace(
            current_waypoint=0,
            distance_remaining=1.5,
            navigation_time=SimpleNamespace(sec=1, nanosec=0),
            estimated_time_remaining=SimpleNamespace(sec=2, nanosec=0),
        )
        self.cancel_count = 0
        self.sent_goals = []

    def waitUntilNav2Active(self, localizer):
        if not self.wait_ready:
            time.sleep(0.05)
            return

    def get_clock(self):
        return FakeClock()

    def goToPose(self, pose):
        if not self.send_ok:
            raise RuntimeError("reject")
        self.sent_goals.append(pose)
        self.task_active = True
        self.complete = False
        self.result = FakeTaskResult.SUCCEEDED

    def isTaskComplete(self):
        return self.complete

    def getFeedback(self):
        return self.feedback

    def cancelTask(self):
        self.cancel_count += 1
        self.task_active = False
        self.complete = True
        self.result = FakeTaskResult.CANCELED

    def getResult(self):
        return self.result


def _make_adapter(fake: FakeNavigator) -> BasicNavigatorAdapter:
    return BasicNavigatorAdapter(
        navigator=fake,
        task_result_constants=FakeTaskResult,
    )


def _make_context() -> MissionExecutionContext:
    spec = MissionSpec(
        mission_id="m1",
        frame_id="map",
        waypoints=[WaypointSpec("P1", 1.0, 2.0, 0.0)],
    )
    return MissionExecutionContext(mission_spec=spec)


@pytest.fixture
def mission_node(ros_context):
    fake = FakeNavigator()
    node = MissionActionServerNode(navigator_factory=lambda: _make_adapter(fake))
    yield node, fake
    node.destroy_node()


def test_goal_callback_reject_when_busy(mission_node):
    node, _ = mission_node
    node._active_goal = True
    assert node._goal_callback(MagicMock()) == rclpy.action.GoalResponse.REJECT


def test_goal_callback_accept(mission_node):
    node, _ = mission_node
    assert node._goal_callback(MagicMock()) == rclpy.action.GoalResponse.ACCEPT
    assert node._active_goal is True
    with node._goal_lock:
        node._active_goal = False


def test_on_safety_state_safe_tracking(mission_node):
    node, _ = mission_node
    msg = SafetyState()
    msg.level = SafetyState.SAFE
    node._on_safety_state(msg)
    assert node._latest_safe_since is not None
    msg.level = SafetyState.STOP_NOW
    node._on_safety_state(msg)
    assert node._latest_safe_since is None


def test_publish_state_and_build_result(mission_node):
    node, _ = mission_node
    ctx = _make_context()
    ctx.state = MissionState.WAITING_FOR_RESULT
    ctx.last_reason = "waiting"
    node._publish_state(ctx)
    result = node._build_result(ctx, success=True)
    assert result.success is True
    assert result.final_state == MissionState.WAITING_FOR_RESULT.value


def test_process_event_nav2_not_ready(mission_node):
    node, fake = mission_node
    machine = MissionStateMachine()
    ctx = _make_context()
    machine.handle_event(ctx, Event(EventType.MISSION_REQUESTED))
    adapter = _make_adapter(fake)
    adapter.wait_until_ready = MagicMock(return_value=False)
    node._process_event(
        machine,
        ctx,
        adapter,
        Event(EventType.MISSION_LOADED, "loaded"),
    )
    assert ctx.state == MissionState.MISSION_FAILED


def test_process_event_send_and_complete_goal(mission_node):
    node, fake = mission_node
    machine = MissionStateMachine()
    ctx = _make_context()
    machine.handle_event(ctx, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(ctx, Event(EventType.MISSION_LOADED))
    node._process_event(
        machine,
        ctx,
        _make_adapter(fake),
        Event(EventType.NAV2_READY, "ready"),
    )
    assert ctx.state == MissionState.WAITING_FOR_RESULT
    assert len(fake.sent_goals) == 1


def test_process_event_cancel_goal(mission_node):
    node, fake = mission_node
    machine = MissionStateMachine()
    ctx = _make_context()
    ctx.state = MissionState.WAITING_FOR_RESULT
    adapter = _make_adapter(fake)
    adapter.send_goal(ctx.current_waypoint)
    node._process_event(
        machine,
        ctx,
        adapter,
        Event(EventType.SAFETY_STOP, "stop"),
    )
    assert ctx.state == MissionState.PAUSED_FOR_SAFETY
    assert fake.cancel_count >= 1


def test_publish_action_feedback(mission_node):
    node, _ = mission_node
    ctx = _make_context()
    ctx.current_feedback = MissionFeedbackSnapshot(distance_remaining=2.5)
    handle = MagicMock()
    node._publish_action_feedback(handle, ctx, time.monotonic())
    handle.publish_feedback.assert_called_once()


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_execute_mission_success(_sleep, mission_node, sample_mission_file):
    node, fake = mission_node
    goal_handle = MagicMock()
    goal_handle.request.mission_file = str(sample_mission_file)
    goal_handle.request.max_retry_per_waypoint = 1
    goal_handle.request.allow_skip_waypoint = False
    goal_handle.is_cancel_requested = False

    call_count = {"n": 0}

    def complete_after_send():
        call_count["n"] += 1
        return call_count["n"] >= 2

    fake.isTaskComplete = complete_after_send

    result = node._execute_callback(goal_handle)
    assert result.success is True
    goal_handle.succeed.assert_called_once()


def test_execute_invalid_mission(mission_node, tmp_path):
    node, _ = mission_node
    bad = tmp_path / "bad.json"
    bad.write_text("not json", encoding="utf-8")

    goal_handle = MagicMock()
    goal_handle.request.mission_file = str(bad)
    goal_handle.request.max_retry_per_waypoint = 1
    goal_handle.request.allow_skip_waypoint = False

    result = node._execute_callback(goal_handle)
    assert result.success is False
    goal_handle.abort.assert_called_once()


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_execute_safety_stop_and_clear(_sleep, mission_node, sample_mission_file):
    node, fake = mission_node
    machine = MissionStateMachine()
    ctx = _make_context()
    machine.handle_event(ctx, Event(EventType.MISSION_REQUESTED))
    adapter = _make_adapter(fake)
    adapter.wait_until_ready = MagicMock(return_value=True)

    node._process_event(
        machine, ctx, adapter, Event(EventType.MISSION_LOADED, "loaded")
    )
    assert ctx.state == MissionState.WAITING_FOR_RESULT

    stop_msg = SafetyState()
    stop_msg.level = SafetyState.STOP_NOW
    stop_msg.reason = "blocked"
    node._on_safety_state(stop_msg)

    node._process_event(
        machine, ctx, adapter, Event(EventType.SAFETY_STOP, stop_msg.reason)
    )
    assert ctx.state == MissionState.PAUSED_FOR_SAFETY

    safe_msg = SafetyState()
    safe_msg.level = SafetyState.SAFE
    node._on_safety_state(safe_msg)
    with node._safety_lock:
        node._latest_safe_since = time.monotonic() - 5.0

    node._process_event(
        machine, ctx, adapter, Event(EventType.SAFETY_CLEAR, "clear")
    )
    assert ctx.state == MissionState.WAITING_FOR_RESULT


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_execute_goal_timeout(_sleep, mission_node, sample_mission_file):
    node, fake = mission_node
    machine = MissionStateMachine()
    ctx = _make_context()
    ctx.max_retry_per_waypoint = 0
    ctx.allow_skip_waypoint = False
    machine.handle_event(ctx, Event(EventType.MISSION_REQUESTED))
    adapter = _make_adapter(fake)
    adapter.wait_until_ready = MagicMock(return_value=True)
    node._process_event(machine, ctx, adapter, Event(EventType.MISSION_LOADED, "loaded"))
    ctx.current_goal_start_monotonic = 0.0
    with patch(
        "nav2_mission_manager.mission_action_server.time.monotonic",
        return_value=500.0,
    ):
        node._process_event(
            machine,
            ctx,
            adapter,
            Event(EventType.GOAL_TIMEOUT, "Goal execution timed out."),
        )
    assert ctx.state == MissionState.MISSION_FAILED


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_execute_client_cancel(_sleep, mission_node, sample_mission_file):
    node, fake = mission_node
    machine = MissionStateMachine()
    ctx = _make_context()
    machine.handle_event(ctx, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(ctx, Event(EventType.MISSION_LOADED))
    machine.handle_event(ctx, Event(EventType.NAV2_READY))
    machine.handle_event(ctx, Event(EventType.GOAL_SENT))
    adapter = _make_adapter(fake)
    node._process_event(
        machine,
        ctx,
        adapter,
        Event(EventType.ACTION_CANCEL_REQUESTED, "user cancel"),
    )
    assert ctx.state == MissionState.MISSION_CANCELED


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_execute_nav_result_failed(_sleep, mission_node, sample_mission_file):
    node, fake = mission_node
    machine = MissionStateMachine()
    ctx = _make_context()
    ctx.max_retry_per_waypoint = 0
    machine.handle_event(ctx, Event(EventType.MISSION_REQUESTED))
    adapter = _make_adapter(fake)
    adapter.wait_until_ready = MagicMock(return_value=True)
    node._process_event(machine, ctx, adapter, Event(EventType.MISSION_LOADED, "loaded"))
    node._process_event(
        machine,
        ctx,
        adapter,
        Event(EventType.GOAL_FAILED, "planner failed"),
    )
    assert ctx.state == MissionState.MISSION_FAILED


def test_mission_manager_node_exports_main():
    from nav2_mission_manager.mission_manager_node import main

    assert callable(main)


def test_cancel_callback(mission_node):
    node, _ = mission_node
    assert node._cancel_callback(MagicMock()) == rclpy.action.CancelResponse.ACCEPT


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_execute_callback_client_cancel_in_loop(_sleep, mission_node, sample_mission_file):
    node, _fake = mission_node
    goal_handle = MagicMock()
    goal_handle.request.mission_file = str(sample_mission_file)
    goal_handle.request.max_retry_per_waypoint = 1
    goal_handle.request.allow_skip_waypoint = False
    goal_handle.is_cancel_requested = True

    result = node._execute_callback(goal_handle)
    assert result.success is False
    assert goal_handle.canceled.called


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_execute_callback_main_loop_branches(_sleep, mission_node, sample_mission_file):
    node, fake = mission_node
    goal_handle = MagicMock()
    goal_handle.request.mission_file = str(sample_mission_file)
    goal_handle.request.max_retry_per_waypoint = 1
    goal_handle.request.allow_skip_waypoint = False
    goal_handle.is_cancel_requested = False

    loop_count = {"n": 0}
    fake.complete = False

    def fake_ok():
        loop_count["n"] += 1
        if loop_count["n"] == 3:
            stop = SafetyState()
            stop.level = SafetyState.STOP_NOW
            stop.reason = "pause"
            node._on_safety_state(stop)
        if loop_count["n"] == 5:
            safe = SafetyState()
            safe.level = SafetyState.SAFE
            node._on_safety_state(safe)
            with node._safety_lock:
                node._latest_safe_since = time.monotonic() - 5.0
        if loop_count["n"] >= 6:
            fake.complete = True
        return loop_count["n"] < 12

    with patch("nav2_mission_manager.mission_action_server.rclpy.ok", side_effect=fake_ok):
        result = node._execute_callback(goal_handle)

    assert result.success is True
    assert result.completed_waypoints >= 1


def test_safety_state_concurrent_read_write(mission_node):
    node, _ = mission_node
    stop = threading.Event()

    def writer() -> None:
        while not stop.is_set():
            msg = SafetyState()
            msg.level = SafetyState.SAFE
            node._on_safety_state(msg)

    def reader() -> None:
        while not stop.is_set():
            msg, since = node._get_safety_snapshot()
            if msg is not None and msg.level == SafetyState.SAFE:
                assert since is not None

    threads = [threading.Thread(target=writer), threading.Thread(target=reader)]
    for thread in threads:
        thread.start()
    time.sleep(0.05)
    stop.set()
    for thread in threads:
        thread.join(timeout=1.0)


def test_goal_lock_prevents_double_accept(mission_node):
    node, _ = mission_node
    assert node._goal_callback(MagicMock()) == rclpy.action.GoalResponse.ACCEPT
    assert node._goal_callback(MagicMock()) == rclpy.action.GoalResponse.REJECT
    with node._goal_lock:
        node._active_goal = False


def test_should_backup_for_safety(mission_node):
    node, _ = mission_node
    assert node._should_backup_for_safety() is False

    msg = SafetyState()
    msg.level = SafetyState.STOP_NOW
    msg.state_label = "PAUSED"
    node._on_safety_state(msg)
    assert node._should_backup_for_safety() is True


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_publish_stop(_sleep, mission_node):
    node, _ = mission_node
    node._publish_stop(count=2, period_sec=0.01)


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
@patch("nav2_mission_manager.mission_action_server.rclpy.ok", return_value=True)
def test_perform_safety_backup_and_replan(_ok, _sleep, mission_node):
    node, _ = mission_node
    msg = SafetyState()
    msg.level = SafetyState.STOP_NOW
    msg.state_label = "PAUSED"
    node._on_safety_state(msg)

    for client in node._clear_costmap_clients.values():
        client.wait_for_service = MagicMock(return_value=True)
        future = MagicMock()
        future.done.return_value = True
        client.call_async = MagicMock(return_value=future)

    node._perform_safety_backup_and_replan_prep()


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_clear_costmaps_service_unavailable(_sleep, mission_node):
    node, _ = mission_node
    for client in node._clear_costmap_clients.values():
        client.wait_for_service = MagicMock(return_value=False)
    node._clear_costmaps_for_replan()


@patch("nav2_mission_manager.mission_action_server.time.sleep", return_value=None)
def test_cancel_goal_waits_for_task_complete(_sleep, mission_node):
    node, fake = mission_node
    machine = MissionStateMachine()
    ctx = _make_context()
    ctx.state = MissionState.WAITING_FOR_RESULT
    adapter = _make_adapter(fake)
    adapter.send_goal(ctx.current_waypoint)
    fake.task_active = True
    fake.complete = True

    msg = SafetyState()
    msg.level = SafetyState.STOP_NOW
    msg.state_label = "PAUSED"
    node._on_safety_state(msg)

    node._process_event(
        machine,
        ctx,
        adapter,
        Event(EventType.SAFETY_STOP, "stop"),
    )
    assert ctx.state == MissionState.PAUSED_FOR_SAFETY


@patch("nav2_mission_manager.mission_action_server.MultiThreadedExecutor")
@patch("nav2_mission_manager.mission_action_server.MissionActionServerNode")
@patch("nav2_mission_manager.mission_action_server.rclpy")
def test_main_keyboard_interrupt(mock_rclpy, mock_node_cls, mock_executor_cls):
    from nav2_mission_manager.mission_action_server import main

    mock_rclpy.ok.return_value = True
    mock_executor = mock_executor_cls.return_value
    mock_executor.spin.side_effect = KeyboardInterrupt

    main()

    mock_executor.shutdown.assert_called_once()
