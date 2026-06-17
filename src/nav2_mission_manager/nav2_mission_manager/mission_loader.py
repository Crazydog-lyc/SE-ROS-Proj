# ========================================================================
# 文件: src/nav2_mission_manager/nav2_mission_manager/mission_loader.py
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
from pathlib import Path

from .models import MissionSpec, WaypointSpec



class MissionLoadError(ValueError):
    """mission JSON 格式不对时抛出，上层转成 MISSION_INVALID。"""


def _require_numeric(value: object, field_name: str) -> float:
    # 坐标字段必须是数字，否则后面 Nav2 会 silently 发错点
    if not isinstance(value, (int, float)):
        raise MissionLoadError(f"Field '{field_name}' must be numeric.")
    return float(value)


def load_mission_file(mission_file: str) -> MissionSpec:
    # 支持相对路径，batch 生成的 case 目录里常用
    path = Path(mission_file)
    if not path.exists():
        raise MissionLoadError(f"Mission file does not exist: {path}")

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise MissionLoadError(f"Mission file is not valid JSON: {exc}") from exc

    mission_id = data.get("mission_id")
    if not isinstance(mission_id, str) or not mission_id.strip():
        raise MissionLoadError("Mission file requires a non-empty 'mission_id'.")

    frame_id = data.get("frame_id", "map")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise MissionLoadError("Field 'frame_id' must be a non-empty string.")

    waypoints_data = data.get("waypoints")
    if not isinstance(waypoints_data, list) or not waypoints_data:
        raise MissionLoadError("Mission file requires a non-empty 'waypoints' list.")

    waypoints: list[WaypointSpec] = []
    for index, waypoint in enumerate(waypoints_data):
        if not isinstance(waypoint, dict):
            raise MissionLoadError(f"Waypoint #{index} must be an object.")

        waypoint_id = waypoint.get("id")
        if not isinstance(waypoint_id, str) or not waypoint_id.strip():
            raise MissionLoadError(f"Waypoint #{index} requires a non-empty 'id'.")

        waypoints.append(
            WaypointSpec(
                waypoint_id=waypoint_id,
                x=_require_numeric(waypoint.get("x"), f"waypoints[{index}].x"),
                y=_require_numeric(waypoint.get("y"), f"waypoints[{index}].y"),
                yaw=_require_numeric(waypoint.get("yaw"), f"waypoints[{index}].yaw"),
            )
        )

    return MissionSpec(mission_id=mission_id, frame_id=frame_id, waypoints=waypoints)
