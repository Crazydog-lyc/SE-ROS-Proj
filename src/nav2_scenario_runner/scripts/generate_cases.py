#!/usr/bin/env python3
import argparse
import csv
import json
import pathlib
import subprocess
import sys
from typing import Any, Dict, List

from ament_index_python.packages import get_package_share_directory
import yaml

GENERATOR_MAP = {
    "corridor": "nav2_scenario_runner::plugins::CorridorGenerator",
    "room_inspection": "nav2_scenario_runner::plugins::RoomInspectionGenerator",
    "congestion": "nav2_scenario_runner::plugins::CongestionGenerator",
    "fault_injection": "nav2_scenario_runner::plugins::FaultInjectionGenerator",
}

def build_ros_args(case: Dict[str, Any], output_dir: pathlib.Path) -> List[str]:
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


def _write_mission_file(case_id: str, waypoints_path: pathlib.Path, output_dir: pathlib.Path) -> pathlib.Path:
    payload = _load_yaml(waypoints_path)
    mission = {
        "mission_id": f"{case_id}_mission",
        "frame_id": "map",
        "waypoints": [],
    }
    for index, waypoint in enumerate(payload.get("waypoints", []), start=1):
        mission["waypoints"].append(
            {
                "id": f"W{index}",
                "x": float(waypoint["x"]),
                "y": float(waypoint["y"]),
                "yaw": float(waypoint.get("yaw", 0.0)),
            }
        )

    mission_path = output_dir / f"{case_id}_mission.json"
    with open(mission_path, "w", encoding="utf-8") as handle:
        json.dump(mission, handle, indent=2)
    return mission_path


def _build_semantic_zone(region: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "enabled": bool(region.get("enabled", True)),
        "mode": "all",
        "shape": "rectangle",
        "x": float(region.get("x", 0.0)),
        "y": float(region.get("y", 0.0)),
        "width": float(region.get("width", 0.0)),
        "height": float(region.get("height", 0.0)),
        "yaw": float(region.get("yaw", 0.0)),
        "cost": int(region.get("cost", 0)),
        "apply_to_unknown": False,
    }


def _write_semantic_overlay(case_id: str, semantic_path: pathlib.Path, output_dir: pathlib.Path) -> pathlib.Path:
    semantic_share = pathlib.Path(get_package_share_directory("semantic_costmap_plugins"))
    base_config = _load_yaml(semantic_share / "config" / "nav2_params_semantic.yaml")
    semantic_payload = _load_yaml(semantic_path)

    layer = base_config["global_costmap"]["global_costmap"]["ros__parameters"]["semantic_zone_layer"]
    zone_names: List[str] = []
    zones: Dict[str, Dict[str, Any]] = {}
    for collection_name in ("keepout_regions", "soft_cost_regions"):
        for index, region in enumerate(semantic_payload.get(collection_name, []), start=1):
            zone_name = str(region.get("id") or f"{collection_name}_{index}")
            zone_names.append(zone_name)
            zones[zone_name] = _build_semantic_zone(region)

    layer["zone_names"] = zone_names
    layer["zones"] = zones

    overlay_path = output_dir / f"{case_id}_semantic_overlay.yaml"
    with open(overlay_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(base_config, handle, sort_keys=False)
    return overlay_path


def write_companion_files(case_id: str, output_dir: pathlib.Path) -> None:
    waypoints_path = output_dir / f"{case_id}_waypoints.yaml"
    semantic_path = output_dir / f"{case_id}_semantic_regions.yaml"
    _write_mission_file(case_id, waypoints_path, output_dir)
    _write_semantic_overlay(case_id, semantic_path, output_dir)

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with open(args.profile, "r", encoding="utf-8") as f:
        profile = yaml.safe_load(f)

    cases = profile.get("cases", [])
    if not cases:
        print("No cases found in profile.", file=sys.stderr)
        return 1

    rows = []
    for case in cases:
        print("Generating", case["case_id"])
        result = subprocess.run(build_ros_args(case, output_dir), check=False)
        if result.returncode == 0:
            write_companion_files(case["case_id"], output_dir)
        rows.append({
            "case_id": case["case_id"],
            "scenario_type": case.get("scenario_type", "corridor"),
            "seed": case.get("seed", 42),
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
