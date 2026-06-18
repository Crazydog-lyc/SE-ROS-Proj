#!/usr/bin/python3.10
# ========================================================================
# 文件: src/nav2_scenario_runner/scripts/generate_cases.py
# 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
# ========================================================================
#
# 【AI-PROMPT】
# generate_cases.py：读 batch_profiles/*.yaml，展开 case 矩阵，对每个 case 调 ros2 run
# generate_scenario_node，输出到 --output-dir。请生成 argparse + profile 解析 + subprocess 调用框架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
# 【批跑说明】输出目录结构：cases/<case_id>/{world,mission,metrics}
import argparse
import csv
import json
import math
import pathlib
import subprocess
import sys
from typing import Any, Dict, List

from ament_index_python.packages import get_package_share_directory
import yaml

GENERATOR_MAP = {
    # 字符串类型 -> pluginlib 全名，和 plugins.xml 里一致
    "corridor": "nav2_scenario_runner::plugins::CorridorGenerator",
    "room_inspection": "nav2_scenario_runner::plugins::RoomInspectionGenerator",
    "congestion": "nav2_scenario_runner::plugins::CongestionGenerator",
    "fault_injection": "nav2_scenario_runner::plugins::FaultInjectionGenerator",
}

SAFE_GOAL_X_MIN = 0.6
SAFE_GOAL_X_MAX = 5.0
SAFE_GOAL_Y_MIN = -2.0
SAFE_GOAL_Y_MAX = 2.0
MIN_WAYPOINT_SPACING = 0.45
PILLAR_ROOM_WAYPOINTS = [
    {"x": 1.2, "y": -1.65, "yaw": 0.0},
    {"x": 4.45, "y": 1.25, "yaw": 0.0},
]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _append_goal(
    mission: Dict[str, Any],
    goal: Dict[str, float],
    yaw: float,
    last_goal: Dict[str, float] | None,
) -> Dict[str, float] | None:
    if last_goal is not None:
        last_dx = goal["x"] - last_goal["x"]
        last_dy = goal["y"] - last_goal["y"]
        if (last_dx * last_dx + last_dy * last_dy) < MIN_WAYPOINT_SPACING ** 2:
            return last_goal
    mission["waypoints"].append(
        {
            "id": f"W{len(mission['waypoints']) + 1}",
            "x": goal["x"],
            "y": goal["y"],
            "yaw": yaw,
        }
    )
    return goal


def build_ros_args(case: Dict[str, Any], output_dir: pathlib.Path) -> List[str]:
    # 每个 case 启一个 generate_scenario_node 子进程
    scenario_type = case.get("scenario_type", "corridor")
    plugin = case.get("generator_plugin", GENERATOR_MAP.get(scenario_type, GENERATOR_MAP["corridor"]))
    return [
        "ros2", "run", "nav2_scenario_runner", "generate_scenario_node",
        "--ros-args",
        "-p", f"generator_plugin:={plugin}",
        "-p", f"case_id:={case['case_id']}",
        "-p", f"scenario_type:={scenario_type}",
        "-p", f"seed:={case.get('seed', 42)}",
        "-p", f"waypoint_count:={case.get('waypoint_count', 4)}",
        "-p", f"room_count:={case.get('room_count', 3)}",
        "-p", f"obstacle_density:={case.get('obstacle_density', 2)}",
        "-p", f"enable_semantic_regions:={str(case.get('enable_semantic_regions', True)).lower()}",
        "-p", f"enable_faults:={str(case.get('enable_faults', False)).lower()}",
        "-p", f"enable_congestion:={str(case.get('enable_congestion', False)).lower()}",
        "-p", f"map_width:={case.get('map_width', 8.0)}",
        "-p", f"map_height:={case.get('map_height', 8.0)}",
        "-p", f"output_dir:={output_dir}",
    ]


def _load_yaml(path: pathlib.Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _write_yaml(path: pathlib.Path, payload: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def _write_case_metadata(case: Dict[str, Any], scenario_path: pathlib.Path) -> None:
    world_file = case.get("world_file")
    if not isinstance(world_file, str) or not world_file.strip():
        return

    scenario = _load_yaml(scenario_path)
    metadata = scenario.get("metadata") or {}
    if not isinstance(metadata, dict):
        metadata = {}
    metadata["world_file"] = world_file.strip()
    scenario["metadata"] = metadata
    _write_yaml(scenario_path, scenario)


def _write_mission_file(
    case_id: str,
    scenario_path: pathlib.Path,
    waypoints_path: pathlib.Path,
    output_dir: pathlib.Path,
    case: Dict[str, Any],
) -> pathlib.Path:
    scenario = _load_yaml(scenario_path)
    payload = _load_yaml(waypoints_path)
    mission = {
        "mission_id": f"{case_id}_mission",
        "frame_id": "map",
        "waypoints": [],
    }
    start_pose = scenario.get("start_pose", {}) or {}
    spawn_yaw = float(start_pose.get("yaw", 0.0))
    mission_waypoints = case.get("mission_waypoints") or PILLAR_ROOM_WAYPOINTS
    last_goal = None
    for waypoint in mission_waypoints:
        last_goal = _append_goal(
            mission,
            {
                "x": _clamp(float(waypoint["x"]), SAFE_GOAL_X_MIN, SAFE_GOAL_X_MAX),
                "y": _clamp(float(waypoint["y"]), SAFE_GOAL_Y_MIN, SAFE_GOAL_Y_MAX),
            },
            float(waypoint.get("yaw", 0.0)) - spawn_yaw,
            last_goal,
        )

    if not mission["waypoints"]:
        mission["waypoints"].append({"id": "W1", "x": 0.0, "y": 0.0, "yaw": 0.0})

    mission_path = output_dir / f"{case_id}_mission.json"
    with open(mission_path, "w", encoding="utf-8") as handle:
        json.dump(mission, handle, indent=2)
    return mission_path


def _build_semantic_zone(
    region: Dict[str, Any],
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
) -> Dict[str, Any]:
    world_dx = float(region.get("x", 0.0)) - spawn_x
    world_dy = float(region.get("y", 0.0)) - spawn_y
    cos_yaw = math.cos(-spawn_yaw)
    sin_yaw = math.sin(-spawn_yaw)
    return {
        "enabled": bool(region.get("enabled", True)),
        "mode": "all",
        "shape": "rectangle",
        "x": world_dx * cos_yaw - world_dy * sin_yaw,
        "y": world_dx * sin_yaw + world_dy * cos_yaw,
        "width": float(region.get("width", 0.0)),
        "height": float(region.get("height", 0.0)),
        "yaw": float(region.get("yaw", 0.0)) - spawn_yaw,
        "cost": int(region.get("cost", 0)),
        "apply_to_unknown": False,
    }


def _write_semantic_overlay(
    case_id: str,
    scenario_path: pathlib.Path,
    semantic_path: pathlib.Path,
    output_dir: pathlib.Path,
) -> pathlib.Path:
    semantic_share = pathlib.Path(get_package_share_directory("semantic_costmap_plugins"))
    base_config = _load_yaml(semantic_share / "config" / "nav2_params_semantic.yaml")
    scenario = _load_yaml(scenario_path)
    semantic_payload = _load_yaml(semantic_path)

    start_pose = scenario.get("start_pose", {}) or {}
    spawn_x = float(start_pose.get("x", -2.0))
    spawn_y = float(start_pose.get("y", 0.0))
    spawn_yaw = float(start_pose.get("yaw", 0.0))

    global_params = base_config["global_costmap"]["global_costmap"]["ros__parameters"]
    global_params["rolling_window"] = True
    global_params["width"] = int(math.ceil(float(scenario.get("map_width", 12.0)) + 4.0))
    global_params["height"] = int(math.ceil(float(scenario.get("map_height", 12.0)) + 4.0))
    global_params["track_unknown_space"] = False
    global_params["plugins"] = ["obstacle_layer", "semantic_zone_layer", "inflation_layer"]
    global_params.pop("static_layer", None)

    layer = global_params["semantic_zone_layer"]
    zone_names: List[str] = []
    zones: Dict[str, Dict[str, Any]] = {}
    for collection_name in ("keepout_regions", "soft_cost_regions"):
        for index, region in enumerate(semantic_payload.get(collection_name, []), start=1):
            zone_name = str(region.get("id") or f"{collection_name}_{index}")
            zone_names.append(zone_name)
            zones[zone_name] = _build_semantic_zone(
                region,
                spawn_x,
                spawn_y,
                spawn_yaw,
            )

    layer["zone_names"] = zone_names
    layer["zones"] = zones

    overlay_path = output_dir / f"{case_id}_semantic_overlay.yaml"
    with open(overlay_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(base_config, handle, sort_keys=False)
    return overlay_path


def write_companion_files(case: Dict[str, Any], output_dir: pathlib.Path) -> None:
    case_id = case["case_id"]
    scenario_path = output_dir / f"{case_id}_scenario.yaml"
    waypoints_path = output_dir / f"{case_id}_waypoints.yaml"
    semantic_path = output_dir / f"{case_id}_semantic_regions.yaml"
    _write_case_metadata(case, scenario_path)
    _write_mission_file(case_id, scenario_path, waypoints_path, output_dir, case)
    _write_semantic_overlay(case_id, scenario_path, semantic_path, output_dir)

# 读 profile -> 展开 case 矩阵 -> subprocess 生成
def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with open(args.profile, "r", encoding="utf-8") as f:
        profile = yaml.safe_load(f)

    batch_settings = profile.get("batch_settings") or {}
    cases = profile.get("cases", [])
    if not cases:
        print("No cases found in profile.", file=sys.stderr)
        return 1

    rows = []
    for case in cases:
        case_config = dict(case)
        if "world_file" not in case_config and batch_settings.get("world_file"):
            case_config["world_file"] = batch_settings["world_file"]

        print("Generating", case_config["case_id"])
        result = subprocess.run(build_ros_args(case_config, output_dir), check=False)
        if result.returncode == 0:
            write_companion_files(case_config, output_dir)
        rows.append({
            "case_id": case_config["case_id"],
            "scenario_type": case_config.get("scenario_type", "corridor"),
            "seed": case_config.get("seed", 42),
            "success": result.returncode == 0,
            "return_code": result.returncode,
        })

    with open(output_dir / "manifest.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["case_id", "scenario_type", "seed", "success", "return_code"])
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {output_dir / 'manifest.csv'}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
