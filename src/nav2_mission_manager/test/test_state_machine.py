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
