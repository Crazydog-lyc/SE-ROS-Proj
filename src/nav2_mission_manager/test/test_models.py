import pytest

from nav2_mission_manager.models import (
    MissionExecutionContext,
    MissionFeedbackSnapshot,
    MissionSpec,
    WaypointSpec,
)
from nav2_mission_manager.states import MissionState


def test_context_progress_without_mission():
    ctx = MissionExecutionContext()
    assert ctx.total_waypoints == 0
    assert ctx.progress_percent == 0.0
    assert ctx.mission_id == ""


def test_context_current_waypoint_requires_mission():
    ctx = MissionExecutionContext()
    with pytest.raises(RuntimeError, match="not been loaded"):
        _ = ctx.current_waypoint


def test_context_properties_with_mission():
    spec = MissionSpec(
        mission_id="m1",
        frame_id="map",
        waypoints=[WaypointSpec("P1", 0.0, 0.0, 0.0)],
    )
    ctx = MissionExecutionContext(mission_spec=spec, completed_count=1)
    assert ctx.mission_id == "m1"
    assert ctx.current_waypoint.waypoint_id == "P1"
    assert ctx.progress_percent == 100.0


def test_feedback_snapshot_defaults():
    snap = MissionFeedbackSnapshot()
    assert snap.distance_remaining is None
