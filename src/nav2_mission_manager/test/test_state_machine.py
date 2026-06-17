# ========================================================================
# 文件: src/nav2_mission_manager/test/test_state_machine.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# MissionStateMachine 用显式 _handle_<state> 方法处理 Event，状态含
# IDLE/LOADING/DISPATCHING/WAITING/CANCELING_FOR_SAFETY/PAUSED_FOR_SAFETY
# 等，TransitionCommand 驱动 WAIT_FOR_NAV2/SEND_GOAL/CANCEL_GOAL。请生成类结构和 handle_event 分发框架，各
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
from nav2_mission_manager.events import Event, EventType
from nav2_mission_manager.models import MissionExecutionContext, MissionSpec, WaypointSpec
from nav2_mission_manager.state_machine import MissionStateMachine
from nav2_mission_manager.states import MissionState



def make_context(max_retry_per_waypoint=1, allow_skip_waypoint=False):
    mission = MissionSpec(
        mission_id="mission_a",
        frame_id="map",
        waypoints=[
            WaypointSpec("P1", 1.0, 2.0, 0.0),
            WaypointSpec("P2", 2.0, 3.0, 1.57),
        ],
    )
    return MissionExecutionContext(
        mission_spec=mission,
        max_retry_per_waypoint=max_retry_per_waypoint,
        allow_skip_waypoint=allow_skip_waypoint,
    )


# 状态机单测：
def test_state_machine_success_path():
    machine = MissionStateMachine()
    context = make_context()

    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    machine.handle_event(context, Event(EventType.GOAL_SUCCEEDED))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    transition = machine.handle_event(context, Event(EventType.GOAL_SUCCEEDED))

    assert transition.new_state == MissionState.MISSION_SUCCEEDED
    assert context.completed_count == 2


# 状态机单测：
def test_state_machine_retries_then_fails():
    machine = MissionStateMachine()
    context = make_context(max_retry_per_waypoint=1, allow_skip_waypoint=False)

    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    transition = machine.handle_event(context, Event(EventType.GOAL_FAILED, "first failure"))
    assert transition.new_state == MissionState.RETRYING_GOAL

    machine.handle_event(context, Event(EventType.GOAL_SENT))
    transition = machine.handle_event(context, Event(EventType.GOAL_FAILED, "second failure"))
    assert transition.new_state == MissionState.MISSION_FAILED
    assert context.failed_count == 1


# 状态机单测：
def test_state_machine_safety_pause_and_resume():
    machine = MissionStateMachine()
    context = make_context()

    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    transition = machine.handle_event(context, Event(EventType.SAFETY_STOP, "blocked"))
    assert transition.new_state == MissionState.CANCELING_FOR_SAFETY

    machine.handle_event(context, Event(EventType.ACTION_CANCEL_CONFIRMED))
    transition = machine.handle_event(context, Event(EventType.SAFETY_CLEAR))
    assert transition.new_state == MissionState.DISPATCHING_GOAL


# 状态机单测：
def test_state_machine_skip_after_retry_exhaustion():
    machine = MissionStateMachine()
    context = make_context(max_retry_per_waypoint=0, allow_skip_waypoint=True)

    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    transition = machine.handle_event(context, Event(EventType.GOAL_FAILED, "skip it"))

    assert transition.new_state == MissionState.SKIPPING_GOAL
    assert context.current_index == 1
    assert context.failed_count == 1


def test_state_machine_terminal_state_ignores_events():
    machine = MissionStateMachine()
    context = make_context()
    context.state = MissionState.MISSION_SUCCEEDED
    transition = machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    assert transition.new_state == MissionState.MISSION_SUCCEEDED


def test_state_machine_nav2_not_ready():
    machine = MissionStateMachine()
    context = make_context()
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    transition = machine.handle_event(context, Event(EventType.NAV2_NOT_READY, "timeout"))
    assert transition.new_state == MissionState.MISSION_FAILED


def test_state_machine_mission_invalid():
    machine = MissionStateMachine()
    context = make_context()
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    transition = machine.handle_event(context, Event(EventType.MISSION_INVALID, "bad json"))
    assert transition.new_state == MissionState.MISSION_FAILED


def test_state_machine_cancel_during_waiting():
    machine = MissionStateMachine()
    context = make_context()
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    transition = machine.handle_event(
        context, Event(EventType.ACTION_CANCEL_REQUESTED, "user cancel")
    )
    assert transition.new_state == MissionState.MISSION_CANCELED


def test_state_machine_goal_timeout_and_skip_last_succeeds():
    machine = MissionStateMachine()
    context = make_context(max_retry_per_waypoint=0, allow_skip_waypoint=True)
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    machine.handle_event(context, Event(EventType.GOAL_TIMEOUT, "slow"))
    transition = machine.handle_event(context, Event(EventType.GOAL_SENT))
    assert transition.new_state == MissionState.WAITING_FOR_RESULT
    transition = machine.handle_event(context, Event(EventType.GOAL_SUCCEEDED))
    assert transition.new_state == MissionState.MISSION_SUCCEEDED


def test_state_machine_cancel_during_paused():
    machine = MissionStateMachine()
    context = make_context()
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    machine.handle_event(context, Event(EventType.SAFETY_STOP))
    machine.handle_event(context, Event(EventType.ACTION_CANCEL_CONFIRMED))
    transition = machine.handle_event(
        context, Event(EventType.ACTION_CANCEL_REQUESTED, "abort")
    )
    assert transition.new_state == MissionState.MISSION_CANCELED


def test_state_machine_skip_final_waypoint_succeeds():
    machine = MissionStateMachine()
    context = make_context(max_retry_per_waypoint=0, allow_skip_waypoint=True)
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    context.current_index = 1
    transition = machine.handle_event(context, Event(EventType.GOAL_FAILED, "skip last"))
    assert transition.new_state == MissionState.MISSION_SUCCEEDED


def test_state_machine_ignores_unhandled_events():
    machine = MissionStateMachine()
    context = make_context()
    assert machine.handle_event(context, Event(EventType.GOAL_SENT)).new_state == MissionState.IDLE

    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    assert (
        machine.handle_event(context, Event(EventType.GOAL_SENT)).new_state
        == MissionState.LOADING_MISSION
    )

    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    assert (
        machine.handle_event(context, Event(EventType.GOAL_SENT)).new_state
        == MissionState.WAITING_FOR_NAV2
    )


def test_state_machine_loading_cancel_and_invalid():
    machine = MissionStateMachine()
    context = make_context()
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    t = machine.handle_event(context, Event(EventType.ACTION_CANCEL_REQUESTED, "x"))
    assert t.new_state == MissionState.MISSION_CANCELED
    context.state = MissionState.LOADING_MISSION
    t = machine.handle_event(context, Event(EventType.MISSION_INVALID, "bad"))
    assert t.new_state == MissionState.MISSION_FAILED


def test_state_machine_dispatching_reject_and_cancel():
    machine = MissionStateMachine()
    context = make_context()
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    t = machine.handle_event(context, Event(EventType.GOAL_REJECTED, "no"))
    assert t.new_state == MissionState.RETRYING_GOAL
    context.state = MissionState.DISPATCHING_GOAL
    t = machine.handle_event(context, Event(EventType.ACTION_CANCEL_REQUESTED, "c"))
    assert t.new_state == MissionState.MISSION_CANCELED


def test_state_machine_waiting_cancel_and_timeout():
    machine = MissionStateMachine()
    context = make_context()
    machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    machine.handle_event(context, Event(EventType.MISSION_LOADED))
    machine.handle_event(context, Event(EventType.NAV2_READY))
    machine.handle_event(context, Event(EventType.GOAL_SENT))
    t = machine.handle_event(context, Event(EventType.GOAL_TIMEOUT, "slow"))
    assert t.new_state == MissionState.RETRYING_GOAL
    context.state = MissionState.WAITING_FOR_RESULT
    t = machine.handle_event(context, Event(EventType.ACTION_CANCEL_REQUESTED, "c"))
    assert t.new_state == MissionState.MISSION_CANCELED


def test_state_machine_retry_and_skip_reject():
    machine = MissionStateMachine()
    context = make_context(max_retry_per_waypoint=1)
    context.state = MissionState.RETRYING_GOAL
    t = machine.handle_event(context, Event(EventType.GOAL_SENT))
    assert t.new_state == MissionState.WAITING_FOR_RESULT
    context.state = MissionState.RETRYING_GOAL
    t = machine.handle_event(context, Event(EventType.GOAL_REJECTED, "bad"))
    assert t.new_state == MissionState.RETRYING_GOAL
    context.state = MissionState.SKIPPING_GOAL
    t = machine.handle_event(context, Event(EventType.GOAL_REJECTED, "bad"))
    assert t.new_state == MissionState.MISSION_FAILED


def test_state_machine_canceling_for_safety_client_cancel():
    machine = MissionStateMachine()
    context = make_context()
    context.state = MissionState.CANCELING_FOR_SAFETY
    t = machine.handle_event(context, Event(EventType.ACTION_CANCEL_REQUESTED, "stop"))
    assert t.new_state == MissionState.MISSION_CANCELED


def test_state_machine_paused_client_cancel():
    machine = MissionStateMachine()
    context = make_context()
    context.state = MissionState.PAUSED_FOR_SAFETY
    t = machine.handle_event(context, Event(EventType.ACTION_CANCEL_REQUESTED, "stop"))
    assert t.new_state == MissionState.MISSION_CANCELED


def test_state_machine_terminal_keeps_state():
    machine = MissionStateMachine()
    context = make_context()
    context.state = MissionState.MISSION_FAILED
    context.last_reason = "done"
    t = machine.handle_event(context, Event(EventType.MISSION_REQUESTED))
    assert t.new_state == MissionState.MISSION_FAILED
    assert t.reason == "done"
