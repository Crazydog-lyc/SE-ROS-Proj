import os
import tempfile
from typing import Any, Dict

import yaml
from ament_index_python.packages import get_package_share_directory


def _set_bt_xml_paths(data: Dict[str, Any]) -> None:
    share = get_package_share_directory("sam_bot_nav2_gz")
    bt_dir = os.path.join(share, "config", "behavior_trees")
    bt_nav = data.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})
    bt_nav["default_nav_to_pose_bt_xml"] = os.path.join(
        bt_dir, "navigate_to_pose_w_replanning_and_recovery.xml"
    )
    bt_nav["default_nav_through_poses_bt_xml"] = os.path.join(
        bt_dir, "navigate_through_poses_w_replanning_and_recovery.xml"
    )


def patch_nav2_params(params_file: str) -> str:
    """Inject custom behavior tree paths and return a temp params file."""
    with open(params_file, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    _set_bt_xml_paths(data)

    fd, patched_path = tempfile.mkstemp(suffix=".yaml", prefix="nav2_params_")
    os.close(fd)
    with open(patched_path, "w", encoding="utf-8") as handle:
        yaml.dump(data, handle, sort_keys=False)

    return patched_path
