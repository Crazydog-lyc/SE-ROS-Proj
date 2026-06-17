# ========================================================================
# 文件: src/nav2_mission_manager/test/test_mission_loader.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# load_mission_file(path) 读 JSON mission：mission_id、frame_id、waypoints[{id,x,y,yaw}]，校验失败抛
# MissionLoadError，请生成解析函数和 dataclass 映射骨架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
import json

import pytest

from nav2_mission_manager.mission_loader import MissionLoadError, load_mission_file



# mission JSON 校验
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


# mission JSON 校验
def test_load_mission_file_rejects_empty_waypoints(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(
        json.dumps({"mission_id": "mission_a", "waypoints": []}),
        encoding="utf-8",
    )

    with pytest.raises(MissionLoadError, match="non-empty 'waypoints'"):
        load_mission_file(str(mission_file))


# mission JSON 校验
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


def test_load_mission_file_missing_path():
    with pytest.raises(MissionLoadError, match="does not exist"):
        load_mission_file("/no/such/mission.json")


def test_load_mission_file_invalid_json(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text("{bad", encoding="utf-8")
    with pytest.raises(MissionLoadError, match="not valid JSON"):
        load_mission_file(str(mission_file))


def test_load_mission_file_empty_mission_id(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(
        json.dumps({"mission_id": "  ", "waypoints": [{"id": "P1", "x": 0, "y": 0, "yaw": 0}]}),
        encoding="utf-8",
    )
    with pytest.raises(MissionLoadError, match="mission_id"):
        load_mission_file(str(mission_file))


def test_load_mission_file_invalid_frame_id(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(
        json.dumps(
            {
                "mission_id": "m1",
                "frame_id": "",
                "waypoints": [{"id": "P1", "x": 0, "y": 0, "yaw": 0}],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(MissionLoadError, match="frame_id"):
        load_mission_file(str(mission_file))


def test_load_mission_file_waypoint_not_object(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(
        json.dumps({"mission_id": "m1", "waypoints": ["bad"]}),
        encoding="utf-8",
    )
    with pytest.raises(MissionLoadError, match="must be an object"):
        load_mission_file(str(mission_file))


def test_load_mission_file_empty_waypoint_id(tmp_path):
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(
        json.dumps({"mission_id": "m1", "waypoints": [{"id": "", "x": 0, "y": 0, "yaw": 0}]}),
        encoding="utf-8",
    )
    with pytest.raises(MissionLoadError, match="non-empty 'id'"):
        load_mission_file(str(mission_file))
