import json

import pytest

from nav2_mission_manager.mission_loader import MissionLoadError, load_mission_file


def test_load_mission_file_success(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(
        json.dumps(
            {
                "mission_id": "mission_a",
                "frame_id": "map",
                "waypoints": [
                    {"id": "P1", "x": 1.0, "y": 2.0, "yaw": 0.0},
                    {"id": "P2", "x": 3.0, "y": 4.0, "yaw": 1.57},
                ],
            }
        ),
        encoding="utf-8",
    )

    mission = load_mission_file(str(mission_file))

    assert mission.mission_id == "mission_a"
    assert mission.frame_id == "map"
    assert len(mission.waypoints) == 2
    assert mission.waypoints[0].waypoint_id == "P1"


def test_load_mission_file_rejects_empty_waypoints(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(
        json.dumps({"mission_id": "mission_a", "waypoints": []}),
        encoding="utf-8",
    )

    with pytest.raises(MissionLoadError, match="non-empty 'waypoints'"):
        load_mission_file(str(mission_file))


def test_load_mission_file_rejects_non_numeric_pose(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(
        json.dumps(
            {
                "mission_id": "mission_a",
                "waypoints": [{"id": "P1", "x": "bad", "y": 2.0, "yaw": 0.0}],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(MissionLoadError, match="must be numeric"):
        load_mission_file(str(mission_file))
